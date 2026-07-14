#include "at32wk_uart_hs.hpp"

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
    UARTHS::UARTHS(usart_type* _huart, dma_channel_type* _rx_dma, dma_channel_type* _tx_dma, const bool rs485_mode) :
        UARTHSBase(), huart(_huart), rx_dma(_rx_dma), tx_dma(_tx_dma), is_rs485_mode(rs485_mode) {}

    UARTHS::UARTHS(usart_type* _huart, dma_channel_type* _rx_dma, dma_channel_type* _tx_dma) :
        UARTHS(_huart, _rx_dma, _tx_dma, false) {}

    FuncRetCode UARTHS::Init(DataType::Comm::UARTBaudrate baud)
    {
        usart_reset(huart);
        FuncRetCode ret = tx_fifo.init(TX_FIFO_BUFFER_SIZE);
        if(ret != FuncRetCode::OK) return ret;
        ret = rx_fifo.init(RX_FIFO_BUFFER_SIZE);
        if(ret != FuncRetCode::OK) return ret;
        uint32_t baudrate = 115200;
        // delay: For RS-485 use, TSDT/TCDT, unit: time per a bit
        // For most of the RS-485 transmitter chips, we choose a typical delay value of 120ns. (see datasheet)
        // Example: baud=921600, 1 unit is 67.9ns, then delay should be 2.
        //          baud=4000000, 1 unit is 15.6ns, then delay should be 8.
        uint16_t delay = 1;
        switch(baud)
        {
            case DataType::Comm::UARTBaudrate::BAUD_9600: baudrate = 9600; break;
            case DataType::Comm::UARTBaudrate::BAUD_230400: baudrate = 230400; break;
            case DataType::Comm::UARTBaudrate::BAUD_460800: baudrate = 460800; break;
            case DataType::Comm::UARTBaudrate::BAUD_921600: baudrate = 921600; delay = 2; break;
            case DataType::Comm::UARTBaudrate::BAUD_1843200: baudrate = 1843200; delay = 2; break;
            case DataType::Comm::UARTBaudrate::BAUD_1000000: baudrate = 1000000; delay = 2; break;
            case DataType::Comm::UARTBaudrate::BAUD_4000000: baudrate = 4000000; delay = 8; break;
            case DataType::Comm::UARTBaudrate::BAUD_6000000: baudrate = 6000000; delay = 12; break;
            case DataType::Comm::UARTBaudrate::BAUD_8000000: baudrate = 8000000; delay = 18; break;
            case DataType::Comm::UARTBaudrate::BAUD_9000000: baudrate = 9000000; delay = 22; break;
            default: break;
        }
        dma_channel_enable(rx_dma, FALSE);
        dma_channel_enable(tx_dma, FALSE);
        // RX DMA Settings
        const auto RX_DMA_INDEX = get_dma_index_by_chaddr((uint32_t)rx_dma);
        const auto RX_DMA_CHANNEL = get_channel_index_by_chaddr((uint32_t)rx_dma);
        if(RX_DMA_INDEX > 2 || RX_DMA_INDEX < 1 || RX_DMA_CHANNEL > 7 || RX_DMA_CHANNEL < 1) return FuncRetCode::PARAM_OUT_BOUND;
        RX_DMA_GL_FLAG    = ((RX_DMA_INDEX - 1) << 28) | (1 << (4 * (RX_DMA_CHANNEL - 1)));
        RX_DMA_DTERR_FLAG = ((RX_DMA_INDEX - 1) << 28) | (8 << (4 * (RX_DMA_CHANNEL - 1)));
        rx_dma->ctrl_bit.mincm  = TRUE;
        rx_dma->ctrl_bit.pincm  = FALSE;
        rx_dma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
        rx_dma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_BYTE;
        rx_dma->ctrl_bit.lm     = TRUE; // Circular mode
        rx_dma->dtcnt = rx_buffer.max_size();
        rx_dma->paddr = (uint32_t)&huart->dt;
        rx_dma->maddr = (uint32_t)rx_buffer.data();
        // TX DMA Settings
        const auto TX_DMA_INDEX = get_dma_index_by_chaddr((uint32_t)tx_dma);
        const auto TX_DMA_CHANNEL = get_channel_index_by_chaddr((uint32_t)tx_dma);
        if(TX_DMA_INDEX > 2 || TX_DMA_INDEX < 1 || TX_DMA_CHANNEL > 7 || TX_DMA_CHANNEL < 1) return FuncRetCode::PARAM_OUT_BOUND;
        TX_DMA_GL_FLAG = ((TX_DMA_INDEX - 1) << 28) | (1 << (4 * (TX_DMA_CHANNEL - 1)));
        tx_dma->ctrl_bit.mincm  = TRUE;
        tx_dma->ctrl_bit.pincm  = FALSE;
        tx_dma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
        tx_dma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_BYTE;
        tx_dma->ctrl_bit.lm     = FALSE;
        tx_dma->dtcnt = 0;
        tx_dma->paddr = (uint32_t)&huart->dt;
        tx_buffer.fill(0);
        rx_buffer.fill(0);
        // UART Init
        usart_init(huart, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
        // RS485 mode compatible
        if(is_rs485_mode)
        {
            usart_de_polarity_set(huart, USART_DE_POLARITY_HIGH);
            usart_rs485_delay_time_config(huart, delay, delay);
            usart_rs485_mode_enable(huart, TRUE);
        }
        dma_channel_enable(rx_dma, TRUE);
        usart_receiver_enable(huart, TRUE);
        usart_dma_receiver_enable(huart, TRUE);
        usart_transmitter_enable(huart, TRUE);
        usart_dma_transmitter_enable(huart, TRUE);
        usart_interrupt_enable(huart, USART_IDLE_INT, TRUE);
        usart_enable(huart, TRUE);
        return FuncRetCode::OK;
    }

    void UARTHS::UpdateRxFIFO()
    {
        if(dma_flag_get(RX_DMA_DTERR_FLAG) != RESET)
        {
            dma_flag_clear(RX_DMA_GL_FLAG);
            last_dma_rx_size = 0;
            rx_dma->dtcnt = rx_buffer.max_size();
            rx_dma->paddr = (uint32_t)&huart->dt;
            rx_dma->maddr = (uint32_t)rx_buffer.data();
            dma_channel_enable(rx_dma, TRUE);
            return;
        }
        if(rx_copy_busy) return;

        const uint16_t curr_rx_pos = rx_buffer.max_size() - rx_dma->dtcnt;
        if(curr_rx_pos >= last_dma_rx_size)
        {
            const uint16_t recv_size = curr_rx_pos - last_dma_rx_size;
            if(recv_size > 0) rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        }
        else
        {
            // DMA wrapped: copy tail segment then head segment
            const uint16_t tail = rx_buffer.max_size() - last_dma_rx_size;
            rx_fifo.put(rx_buffer.data() + last_dma_rx_size, tail);
            if(curr_rx_pos > 0) rx_fifo.put(rx_buffer.data(), curr_rx_pos);
        }
        last_dma_rx_size = curr_rx_pos;
    }

    void UARTHS::OnUARTIRQ()
    {
        if(!(huart->sts & USART_IDLEF_FLAG)) return;

        // Step 1: RX tail copy — captures bytes since the last UpdateRxFIFO() or OnUARTIRQ() call.
        // rx_copy_busy prevents UpdateRxFIFO() (higher-priority caller) from racing this section.
        rx_copy_busy = true;
        const uint16_t curr_rx_pos = rx_buffer.max_size() - rx_dma->dtcnt;
        if(curr_rx_pos >= last_dma_rx_size)
        {
            const uint16_t recv_size = curr_rx_pos - last_dma_rx_size;
            if(recv_size > 0) rx_fifo.put(rx_buffer.data() + last_dma_rx_size, recv_size);
        }
        else
        {
            const uint16_t tail = rx_buffer.max_size() - last_dma_rx_size;
            rx_fifo.put(rx_buffer.data() + last_dma_rx_size, tail);
            if(curr_rx_pos > 0) rx_fifo.put(rx_buffer.data(), curr_rx_pos);
        }
        last_dma_rx_size = curr_rx_pos;
        rx_copy_busy = false;

        // Step 2: Invoke idle callback — rx_fifo is fully up to date at this point.
        if(idle_cb) idle_cb(this);

        // Step 3: Start TX DMA if idle and tx_pending — drain all of tx_fifo in one shot.
        // If TX DMA is still running (previous frame not finished), skip.
        if(tx_pending && (tx_dma->ctrl_bit.chen == 0 || tx_dma->dtcnt == 0))
        {
            const auto len = static_cast<uint16_t>(tx_fifo.get(tx_buffer.data(), tx_buffer.max_size()));
            if(len > 0)
            {
                dma_channel_enable(tx_dma, FALSE);
                tx_dma->dtcnt = len;
                tx_dma->maddr = (uint32_t)tx_buffer.data();
                dma_flag_clear(TX_DMA_GL_FLAG);
                dma_channel_enable(tx_dma, TRUE);
            }
            if(tx_fifo.used() == 0) tx_pending = false;
        }

        // Step 4: Clear IDLE flag — AT32 requires reading STS then DT.
        UNUSED(huart->sts);
        UNUSED(huart->dt);
    }

    void UARTHS::StartTransmit()
    {
        if(tx_fifo.used() > 0) tx_pending = true;
    }
}
#endif
