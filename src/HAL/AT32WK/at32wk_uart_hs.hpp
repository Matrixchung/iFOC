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
    void Update() override;
    void StartTransmit() override;
private:
    static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    usart_type *huart;
    dma_channel_type *rx_dma;
    dma_channel_type *tx_dma;
    std::array<uint8_t, TX_FIFO_BUFFER_SIZE> tx_buffer{};
    std::array<uint8_t, RX_FIFO_BUFFER_SIZE> rx_buffer{};
    uint32_t RX_DMA_GL_FLAG = 0;
    uint32_t RX_DMA_FDT_FLAG = 0;
    uint32_t RX_DMA_HDT_FLAG = 0;
    uint32_t RX_DMA_DTERR_FLAG = 0;
    uint32_t TX_DMA_GL_FLAG = 0;
    uint32_t TX_DMA_FDT_FLAG = 0;
    uint32_t TX_DMA_HDT_FLAG = 0;
    uint32_t TX_DMA_DTERR_FLAG = 0;
    uint16_t last_dma_rx_size = 0;
    uint16_t last_rx_dtcnt = 0;    // RS485 idle detection: NDTR snapshot from previous Update()
    // uint8_t RX_DMA_INDEX = 0;
    // uint8_t RX_DMA_CHANNEL = 0;
    // uint8_t TX_DMA_INDEX = 0;
    // uint8_t TX_DMA_CHANNEL = 0;
    bool is_rs485_mode = false;
    bool tx_pending = false;      // tx_fifo has data waiting to be staged into tx_buffer
    uint16_t tx_buffer_len = 0;   // bytes staged in tx_buffer, waiting for bus idle to start DMA
    bool is_tx_busy = false;      // TX DMA is actively transmitting
};
}

#endif