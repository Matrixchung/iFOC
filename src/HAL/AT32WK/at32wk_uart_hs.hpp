#pragma once

#include "uart_hs_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(USART_MODULE_ENABLED)

namespace iFOC::HAL
{
class UARTHS final : public UARTHSBase
{
public:
    UARTHS(usart_type *_huart, dma_channel_type *_rx_dma, dma_channel_type *_tx_dma, bool rs485_mode);
    UARTHS(usart_type *_huart, dma_channel_type *_rx_dma, dma_channel_type *_tx_dma);
    FuncRetCode Init(DataType::Comm::UARTBaudrate baud) override;
    void StartTransmit() override;
    /// Call from the high-frequency periodic interrupt to drain
    /// rx_buffer → rx_fifo mid-transfer, preventing DMA overrun between IDLE events.
    void UpdateRxFIFO() override;
    /// Call from the USART global interrupt handler.
    /// Handles IDLE: copies RX tail, invokes idle_cb, starts TX DMA if idle.
    void OnUARTIRQ();
private:
    // static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 65536) ? 2048 :
    //                                                 (configTOTAL_HEAP_SIZE >= 32768 ? 1024 : 512);
    // static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 65536) ? 2048 :
    //                                                 (configTOTAL_HEAP_SIZE >= 32768 ? 1024 : 512);
    static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384 ? 512 : 256);
    static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384 ? 512 : 256);
    usart_type *huart;
    dma_channel_type *rx_dma;
    dma_channel_type *tx_dma;
    std::array<uint8_t, TX_FIFO_BUFFER_SIZE> tx_buffer{};
    std::array<uint8_t, RX_FIFO_BUFFER_SIZE> rx_buffer{};
    uint32_t RX_DMA_GL_FLAG    = 0;
    uint32_t RX_DMA_DTERR_FLAG = 0;
    uint32_t TX_DMA_GL_FLAG    = 0;
    uint16_t last_dma_rx_size  = 0;
    bool is_rs485_mode         = false;
    bool tx_pending            = false;
    volatile bool rx_copy_busy = false;
};
}

#endif
