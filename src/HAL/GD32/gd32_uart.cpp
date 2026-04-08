#include "gd32_uart.hpp"

#if defined(GD32_ENV)

namespace iFOC::HAL
{
UART::UART(const uint32_t _huart, const uint32_t _rx_dma_inst, const dma_channel_enum _rx_dma_ch,
    const uint32_t _tx_dma_inst, const dma_channel_enum _tx_dma_ch)
    : UARTBase(), huart(_huart), rx_dma_inst(_rx_dma_inst), rx_dma_ch(_rx_dma_ch),
    tx_dma_inst(_tx_dma_inst), tx_dma_ch(_tx_dma_ch) {}

FuncRetCode UART::Init(const DataType::Comm::UARTBaudrate baud)
{
    usart_deinit(huart);
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
    // DMA settings
    dma_deinit(rx_dma_inst, rx_dma_ch);
    dma_deinit(tx_dma_inst, tx_dma_ch);
    dma_parameter_struct dma_rx_init_struct, dma_tx_init_struct;
    // dmamux_sync_parameter_struct dmamux_rx_init_struct, dmamux_tx_init_struct;
    dma_struct_para_init(&dma_rx_init_struct);
    dma_struct_para_init(&dma_tx_init_struct);
    // dmamux_sync_struct_para_init(&dmamux_rx_init_struct);
    // dmamux_sync_struct_para_init(&dmamux_tx_init_struct);

    dma_rx_init_struct.periph_addr = (uint32_t)&USART_RDATA(huart);
    dma_rx_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_rx_init_struct.memory_addr = (uint32_t)rx_buffer.data();
    dma_rx_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_rx_init_struct.number = rx_buffer.max_size();
    dma_rx_init_struct.priority = DMA_PRIORITY_LOW;
    dma_rx_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_rx_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_rx_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;

    dma_tx_init_struct.periph_addr = (uint32_t)&USART_TDATA(huart);
    dma_tx_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_tx_init_struct.memory_addr = (uint32_t)tx_buffer.data();
    dma_tx_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_tx_init_struct.number = 0;
    dma_tx_init_struct.priority = DMA_PRIORITY_LOW;
    dma_tx_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_tx_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_tx_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;

#if defined(GD32G5X3)
    switch(huart)
    {
        case USART0:
        {
            dma_rx_init_struct.request = DMA_REQUEST_USART0_RX;
            dma_tx_init_struct.request = DMA_REQUEST_USART0_TX;
            break;
        }
        case USART1:
        {
            dma_rx_init_struct.request = DMA_REQUEST_USART1_RX;
            dma_tx_init_struct.request = DMA_REQUEST_USART1_TX;
            break;
        }
        case USART2:
        {
            dma_rx_init_struct.request = DMA_REQUEST_USART2_RX;
            dma_tx_init_struct.request = DMA_REQUEST_USART2_TX;
            break;
        }
        case UART3:
        {
            dma_rx_init_struct.request = DMA_REQUEST_UART3_RX;
            dma_tx_init_struct.request = DMA_REQUEST_UART3_TX;
            break;
        }
        case UART4:
        {
            dma_rx_init_struct.request = DMA_REQUEST_UART4_RX;
            dma_tx_init_struct.request = DMA_REQUEST_UART4_TX;
            break;
        }
        default: return FuncRetCode::PARAM_NOT_EXIST;
    }
#else
#error "Target platform USART DMA request not implemented, please implement first"
#endif

    tx_buffer.fill(0);
    rx_buffer.fill(0);

    dma_init(rx_dma_inst, rx_dma_ch, &dma_rx_init_struct);
    dma_circulation_enable(rx_dma_inst, rx_dma_ch);
    dma_memory_to_memory_disable(rx_dma_inst, rx_dma_ch);

    dma_init(tx_dma_inst, tx_dma_ch, &dma_tx_init_struct);
    dma_circulation_disable(tx_dma_inst, tx_dma_ch);
    dma_memory_to_memory_disable(tx_dma_inst, tx_dma_ch);
    dma_channel_disable(tx_dma_inst, tx_dma_ch);

    // UART Init
    usart_oversample_config(huart, USART_OVSMOD_16); // set ovs first
    usart_baudrate_set(huart, baudrate);
    usart_word_length_set(huart, USART_WL_8BIT);
    usart_parity_config(huart, USART_PM_NONE);
    usart_stop_bit_set(huart, USART_STB_1BIT);
    usart_receive_config(huart, USART_RECEIVE_ENABLE);
    usart_transmit_config(huart, USART_TRANSMIT_ENABLE);
    usart_receiver_timeout_disable(huart);
    usart_sample_bit_config(huart, USART_OSB_3BIT);
    usart_fifo_disable(huart);
    usart_overrun_disable(huart);
    usart_reception_error_dma_disable(huart);
    usart_data_first_config(huart, USART_MSBF_LSB);
    usart_invert_config(huart, USART_TXPIN_DISABLE);
    usart_invert_config(huart, USART_RXPIN_DISABLE);
    usart_invert_config(huart, USART_DINV_DISABLE);
#if defined(GD32G5X3)
    usart_invert_config(huart, USART_SWAP_DISABLE);
#endif
    usart_dma_receive_config(huart, USART_RECEIVE_DMA_ENABLE);
    usart_dma_transmit_config(huart, USART_TRANSMIT_DMA_ENABLE);

    event_handler.Start();

    usart_flag_clear(huart, USART_FLAG_IDLE);
    usart_interrupt_enable(huart, USART_INT_IDLE);

    dma_interrupt_enable(rx_dma_inst, rx_dma_ch, DMA_INT_HTF | DMA_INT_FTF);
    dma_interrupt_disable(rx_dma_inst, rx_dma_ch, DMA_INT_ERR);

    dma_interrupt_disable(tx_dma_inst, tx_dma_ch, DMA_INT_HTF | DMA_INT_FTF | DMA_INT_ERR);

    dma_channel_enable(rx_dma_inst, rx_dma_ch);

    state = State::UART_STATE_READY;

    usart_enable(huart);

    return FuncRetCode::OK;
}

FuncRetCode UART::StartTransmit(const bool blocked)
{
    if(!blocked)
    {
        if(xSemaphoreTakeAuto(tx_sem, READ_WRITE_TIMEOUT_MS) == pdTRUE)
        {
            const auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
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
    const auto len = tx_fifo.get(tx_buffer.data(), tx_buffer.max_size());
    return TransmitBlocking(tx_buffer.data(), len, 0xF);
}

void UART::OnUARTIRQ()
{
    if(usart_interrupt_flag_get(huart, USART_INT_FLAG_IDLE))
    {
        const uint16_t recv_total_size = rx_buffer.max_size() - dma_transfer_number_get(rx_dma_inst, rx_dma_ch);
        const uint16_t recv_size = recv_total_size - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = recv_total_size;
        vTaskNotifyGiveFromISR(event_handler.GetHandle(), nullptr);
        usart_interrupt_flag_clear(huart, USART_INT_FLAG_IDLE);
    }
}

void UART::OnRxDMAIRQ()
{
    if(dma_interrupt_flag_get(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_ERR))
    {
        state = State::UART_STATE_READY;
        dma_interrupt_flag_clear(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_ERR);
    }
    if(dma_interrupt_flag_get(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_FTF)) // DMA Full Complete
    {
        const uint16_t recv_size = rx_buffer.max_size() - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = 0;
        dma_interrupt_flag_clear(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_FTF);
    }
    if(dma_interrupt_flag_get(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_HTF)) // DMA Half Complete
    {
        const uint16_t recv_total_size = rx_buffer.max_size() - dma_transfer_number_get(rx_dma_inst, rx_dma_ch);
        const uint16_t recv_size = recv_total_size - last_dma_rx_size;
        rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        last_dma_rx_size = recv_total_size;
        dma_interrupt_flag_clear(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_HTF);
    }
    if(dma_interrupt_flag_get(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_G)) dma_interrupt_flag_clear(rx_dma_inst, rx_dma_ch, DMA_INT_FLAG_G);
}

void UART::OnTxDMAIRQ()
{
    if(dma_interrupt_flag_get(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_ERR))
    {
        state = State::UART_STATE_READY;
        dma_interrupt_flag_clear(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_ERR);
    }
    if(dma_interrupt_flag_get(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_FTF))
    {
        state = State::UART_STATE_READY;
        if(!(DMA_CHCTL(tx_dma_inst, tx_dma_ch) & DMA_CHXCTL_CMEN)) // Normal mode
            dma_channel_disable(tx_dma_inst, tx_dma_ch);
        if(xSemaphoreGiveFromISR(tx_sem, nullptr) != pdTRUE) xSemaphoreTakeFromISR(tx_sem, nullptr);
        dma_interrupt_flag_clear(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_FTF);
    }
    if(dma_interrupt_flag_get(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_HTF)) dma_interrupt_flag_clear(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_HTF);
    if(dma_interrupt_flag_get(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_G)) dma_interrupt_flag_clear(tx_dma_inst, tx_dma_ch, DMA_INT_FLAG_G);
}

FuncRetCode UART::TransmitBlocking(const uint8_t* data, const uint16_t size, const TickType_t timeout)
{
    if(!data || size == 0U) return FuncRetCode::PARAM_NOT_EXIST;
    if(state == State::UART_STATE_READY)
    {
        state = State::UART_STATE_BUSY_TX;
        auto tickstart = xTaskGetTickCount();
        // while(!usart_flag_get(huart, USART_FLAG_TBE))
        while(!(USART_REG_VAL(huart, USART_FLAG_TBE) & BIT(USART_BIT_POS(USART_FLAG_TBE))))
        {
            if(xTaskGetTickCount() - tickstart > timeout)
            {
                state = State::UART_STATE_READY;
                return FuncRetCode::REMOTE_TIMEOUT;
            }
        }
        tickstart = xTaskGetTickCount();
        usart_flag_clear(huart, USART_FLAG_TBE);
        for(uint16_t i = 0; i < size; i++)
        {
            usart_data_transmit(huart, data[i]);
            while(!(USART_REG_VAL(huart, USART_FLAG_TC) & BIT(USART_BIT_POS(USART_FLAG_TC))))
            {
                if(xTaskGetTickCount() - tickstart > timeout)
                {
                    state = State::UART_STATE_READY;
                    return FuncRetCode::REMOTE_TIMEOUT;
                }
            }
            tickstart = xTaskGetTickCount();
            usart_flag_clear(huart, USART_FLAG_TC);
        }
        state = State::UART_STATE_READY;
        return FuncRetCode::OK;
    }
    return FuncRetCode::BUSY;
}

FuncRetCode UART::TransmitDMA(const uint8_t* data, const uint16_t size)
{
    if(!data || size == 0U) return FuncRetCode::PARAM_NOT_EXIST;
    if(state == State::UART_STATE_READY)
    {
        state = State::UART_STATE_BUSY_TX;
        dma_channel_disable(tx_dma_inst, tx_dma_ch);
        dma_transfer_number_config(tx_dma_inst, tx_dma_ch, size);
        dma_periph_address_config(tx_dma_inst, tx_dma_ch, (uint32_t)&USART_TDATA(huart));
        dma_memory_address_config(tx_dma_inst, tx_dma_ch, (uint32_t)data);
        // usart_flag_clear(huart, USART_FLAG_PERR | USART_FLAG_FERR | USART_FLAG_NERR);
        USART_INTC(huart) |= (BIT(USART_BIT_POS(USART_FLAG_PERR))) | (BIT(USART_BIT_POS(USART_FLAG_FERR))) | (BIT(USART_BIT_POS(USART_FLAG_NERR)));
        dma_interrupt_disable(tx_dma_inst, tx_dma_ch, DMA_INT_HTF);
        dma_interrupt_enable(tx_dma_inst, tx_dma_ch, DMA_INT_FTF | DMA_INT_ERR);
        dma_channel_enable(tx_dma_inst, tx_dma_ch);
        return FuncRetCode::OK;
    }
    return FuncRetCode::BUSY;
}
}

#endif