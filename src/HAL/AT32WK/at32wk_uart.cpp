#include "at32wk_uart.hpp"

#if defined(AT32WK_ENV) && defined(USART_MODULE_ENABLED)

constexpr uint8_t get_dma_index_by_chaddr(uint32_t addr)
{
    if(addr > DMA2_CHANNEL7_BASE) return 3;
    if(addr >= DMA2_BASE) return 2;
    if(addr <= DMA1_CHANNEL7_BASE && addr >= DMA1_BASE) return 1;
    return 0;
}

constexpr uint8_t get_channel_index_by_chaddr(uint32_t addr)
{
    auto dma_index = get_dma_index_by_chaddr(addr);
    if(dma_index == 1)
    {
        const auto offset = DMA1_CHANNEL2_BASE - DMA1_CHANNEL1_BASE;
        return ((addr - DMA1_CHANNEL1_BASE) / offset) + 1;
    }
    if(dma_index == 2)
    {
        const auto offset = DMA2_CHANNEL2_BASE - DMA2_CHANNEL1_BASE;
        return ((addr - DMA2_CHANNEL1_BASE) / offset) + 1;
    }
    return 0;
}

namespace iFOC::HAL
{
UART::UART(usart_type *_huart, dma_channel_type *_rx_dma, dma_channel_type *_tx_dma) : UARTBase(), huart(_huart), rx_dma(_rx_dma), tx_dma(_tx_dma) {}


FuncRetCode UART::Init(DataType::Comm::UARTBaudrate baud)
{
    FuncRetCode ret = tx_fifo.init(TX_FIFO_BUFFER_SIZE);
    if(ret != FuncRetCode::OK) return ret;
    ret = rx_fifo.init(RX_FIFO_BUFFER_SIZE);
    if(ret != FuncRetCode::OK) return ret;
    xSemaphoreGive(tx_sem);
    uint32_t baudrate = 115200;
    switch(baud)
    {
        case DataType::Comm::UARTBaudrate::BAUD_9600: baudrate = 9600; break;
        case DataType::Comm::UARTBaudrate::BAUD_230400: baudrate = 230400; break;
        case DataType::Comm::UARTBaudrate::BAUD_460800: baudrate = 460800; break;
        case DataType::Comm::UARTBaudrate::BAUD_921600: baudrate = 921600; break;
        case DataType::Comm::UARTBaudrate::BAUD_1843200: baudrate = 1843200; break;
        default: break;
    }
    dma_channel_enable(rx_dma, FALSE);
    dma_channel_enable(tx_dma, FALSE);
    // RX DMA Settings
    RX_DMA_INDEX = get_dma_index_by_chaddr((uint32_t)rx_dma);
    RX_DMA_CHANNEL = get_channel_index_by_chaddr((uint32_t)rx_dma);
    if(RX_DMA_INDEX > 2 || RX_DMA_INDEX < 1 || RX_DMA_CHANNEL > 7 || RX_DMA_CHANNEL < 1) return FuncRetCode::PARAM_OUT_BOUND;
    rx_dma->ctrl_bit.mincm = TRUE;
    rx_dma->ctrl_bit.pincm = FALSE;
    rx_dma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    rx_dma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_BYTE;
    rx_dma->ctrl_bit.lm = TRUE; // Circular mode
    rx_dma->dtcnt = rx_buffer.max_size();
    rx_dma->paddr = (uint32_t)&huart->dt;
    rx_dma->maddr = (uint32_t)rx_buffer.data();
    // TX DMA Settings
    TX_DMA_INDEX = get_dma_index_by_chaddr((uint32_t)tx_dma);
    TX_DMA_CHANNEL = get_channel_index_by_chaddr((uint32_t)tx_dma);
    if(TX_DMA_INDEX > 2 || TX_DMA_INDEX < 1 || TX_DMA_CHANNEL > 7 || TX_DMA_CHANNEL < 1) return FuncRetCode::PARAM_OUT_BOUND;
    tx_dma->ctrl_bit.mincm = TRUE;
    tx_dma->ctrl_bit.pincm = FALSE;
    tx_dma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    tx_dma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_BYTE;
    tx_dma->ctrl_bit.lm = FALSE;
    tx_dma->dtcnt = 0;
    tx_buffer.fill(0);
    rx_buffer.fill(0);
    // UART Init
    usart_init(huart, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    event_handler.Start();
    dma_channel_enable(rx_dma, TRUE);
    usart_receiver_enable(huart, TRUE);
    usart_dma_receiver_enable(huart, TRUE);
    usart_transmitter_enable(huart, TRUE);
    usart_dma_transmitter_enable(huart, TRUE);
    usart_interrupt_enable(huart, USART_IDLE_INT, TRUE);
    dma_interrupt_enable(rx_dma, DMA_FDT_INT | DMA_HDT_INT, TRUE);
    dma_interrupt_enable(rx_dma, DMA_DTERR_INT, FALSE);
    usart_enable(huart, TRUE);
    return FuncRetCode::OK;
}

FuncRetCode UART::StartTransmit(bool blocked)
{
    if(!blocked)
    {
        if(xSemaphoreTakeAuto(tx_sem, READ_WRITE_TIMEOUT_MS) == pdTRUE)
        {
            auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
            if(len == 0) xSemaphoreGiveAuto(tx_sem);
            else if(TransmitDMA(tx_buffer.data(), len) != FuncRetCode::OK)
            {
                // BUSY, give back semaphore
                xSemaphoreGiveAuto(tx_sem);
                return FuncRetCode::BUSY;
            }
            return FuncRetCode::OK;
        }
        return FuncRetCode::REMOTE_TIMEOUT;
    }
    auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
    return TransmitBlocking(tx_buffer.data(), len, 0xF);
}

void UART::OnUARTIRQ()
{
    if(usart_interrupt_flag_get(huart, USART_IDLEF_FLAG) != RESET)
    {
        uint16_t recv_total_size = rx_buffer.max_size() - rx_dma->dtcnt;
        uint16_t recv_size = recv_total_size - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = recv_total_size;
        vTaskNotifyGiveFromISR(event_handler.GetHandle(), nullptr);
        usart_flag_clear(huart, USART_IDLEF_FLAG);
    }
}

void UART::OnRxDMAIRQ()
{
    const uint32_t GL_FLAG = ((RX_DMA_INDEX - 1) << 28) | (1 << (4 * (RX_DMA_CHANNEL - 1)));
    const uint32_t FDT_FLAG = ((RX_DMA_INDEX - 1) << 28) | (2 << (4 * (RX_DMA_CHANNEL - 1)));
    const uint32_t HDT_FLAG = ((RX_DMA_INDEX - 1) << 28) | (4 << (4 * (RX_DMA_CHANNEL - 1)));
    const uint32_t DTERR_FLAG = ((RX_DMA_INDEX - 1) << 28) | (8 << (4 * (RX_DMA_CHANNEL - 1)));
    if(dma_interrupt_flag_get(DTERR_FLAG) != RESET)
    {
        state = State::UART_STATE_READY;
        dma_flag_clear(DTERR_FLAG);
    }
    if(dma_interrupt_flag_get(FDT_FLAG) != RESET) // DMA Full Complete, ~HAL_UART_RXEVENT_TC
    {
        uint16_t recv_size = rx_buffer.max_size() - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = 0;
        dma_flag_clear(FDT_FLAG);
    }
    if(dma_interrupt_flag_get(HDT_FLAG) != RESET) // DMA Half Complete, ~HAL_UART_RXEVENT_HT
    {
        uint16_t recv_total_size = rx_buffer.max_size() - rx_dma->dtcnt;
        uint16_t recv_size = recv_total_size - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = recv_total_size;
        dma_flag_clear(HDT_FLAG);
    }
    if(dma_interrupt_flag_get(GL_FLAG)) dma_flag_clear(GL_FLAG);
}

void UART::OnTxDMAIRQ()
{
    const uint32_t GL_FLAG = ((TX_DMA_INDEX - 1) << 28) | (1 << (4 * (TX_DMA_CHANNEL - 1)));
    const uint32_t FDT_FLAG = ((TX_DMA_INDEX - 1) << 28) | (2 << (4 * (TX_DMA_CHANNEL - 1)));
    const uint32_t HDT_FLAG = ((TX_DMA_INDEX - 1) << 28) | (4 << (4 * (TX_DMA_CHANNEL - 1)));
    const uint32_t DTERR_FLAG = ((TX_DMA_INDEX - 1) << 28) | (8 << (4 * (TX_DMA_CHANNEL - 1)));
    if(dma_interrupt_flag_get(DTERR_FLAG) != RESET)
    {
        state = State::UART_STATE_READY;
        dma_flag_clear(DTERR_FLAG);
    }
    if(dma_interrupt_flag_get(FDT_FLAG) != RESET)
    {
        state = State::UART_STATE_READY;
        if(tx_dma->ctrl_bit.lm == FALSE) // DMA Normal Mode
        {
            dma_channel_enable(tx_dma, FALSE);
            if(xSemaphoreGiveFromISR(tx_sem, nullptr) != pdTRUE) xSemaphoreTakeFromISR(tx_sem, nullptr);
        }
        else // DMA Circular Mode
        {
            if(xSemaphoreGiveFromISR(tx_sem, nullptr) != pdTRUE) xSemaphoreTakeFromISR(tx_sem, nullptr);
        }
        dma_flag_clear(FDT_FLAG);
    }
    if(dma_interrupt_flag_get(HDT_FLAG) != RESET) dma_flag_clear(HDT_FLAG);
    if(dma_interrupt_flag_get(GL_FLAG) != RESET) dma_flag_clear(GL_FLAG);
}

FuncRetCode UART::TransmitBlocking(uint8_t* data, const uint16_t size, const TickType_t timeout)
{
    if(data == nullptr || size == 0U) return FuncRetCode::PARAM_NOT_EXIST;
    if(state == State::UART_STATE_READY)
    {
        state = State::UART_STATE_BUSY_TX;
        auto tickstart = xTaskGetTickCount();
        while(usart_flag_get(huart, USART_TDBE_FLAG) == RESET)
        {
            if(xTaskGetTickCount() - tickstart > timeout)
            {
                state = State::UART_STATE_READY;
                return FuncRetCode::REMOTE_TIMEOUT;
            }
        }
        tickstart = xTaskGetTickCount();
        usart_flag_clear(huart, USART_TDBE_FLAG);
        for(uint16_t i = 0; i < size; i++)
        {
            usart_data_transmit(huart, (uint16_t)data[i]);
            while(usart_flag_get(huart, USART_TDC_FLAG) == RESET)
            {
                if(xTaskGetTickCount() - tickstart > timeout)
                {
                    state = State::UART_STATE_READY;
                    return FuncRetCode::REMOTE_TIMEOUT;
                }
            }
            tickstart = xTaskGetTickCount();
            usart_flag_clear(huart, USART_TDC_FLAG);
        }
        state = State::UART_STATE_READY;
        return FuncRetCode::OK;
    }
    return FuncRetCode::BUSY;
}

FuncRetCode UART::TransmitDMA(uint8_t* data, const uint16_t size)
{
    if(data == nullptr || size == 0U) return FuncRetCode::PARAM_NOT_EXIST;
    if(state == State::UART_STATE_READY)
    {
        state = State::UART_STATE_BUSY_TX;
        dma_channel_enable(tx_dma, FALSE);
        tx_dma->dtcnt = size;
        tx_dma->paddr = (uint32_t)&huart->dt;
        tx_dma->maddr = (uint32_t)data;
        usart_flag_clear(huart, USART_PERR_FLAG | USART_FERR_FLAG | USART_NERR_FLAG);
        dma_interrupt_enable(tx_dma, DMA_HDT_INT, FALSE);
        dma_interrupt_enable(tx_dma, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
        dma_channel_enable(tx_dma, TRUE);
        return FuncRetCode::OK;
    }
    return FuncRetCode::BUSY;
}
}

#endif