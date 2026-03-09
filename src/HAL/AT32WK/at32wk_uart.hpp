#pragma once

#include "uart_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(USART_MODULE_ENABLED)

namespace iFOC::HAL
{
class UART final : public UARTBase
{
public:
    UART(usart_type *_huart, dma_channel_type *_rx_dma, dma_channel_type *_tx_dma);
    FuncRetCode Init(DataType::Comm::UARTBaudrate baud) override;
    FuncRetCode StartTransmit(bool blocked) override;
    void OnUARTIRQ();
    void OnRxDMAIRQ();
    void OnTxDMAIRQ();
private:
    enum class State : uint16_t
    {
        UART_STATE_READY,
        UART_STATE_BUSY_TX,
        UART_STATE_BUSY_RX
    };
    FuncRetCode TransmitBlocking(uint8_t* data, uint16_t size, TickType_t timeout);
    FuncRetCode TransmitDMA(uint8_t* data, uint16_t size);
    static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    usart_type *huart;
    dma_channel_type *rx_dma;
    dma_channel_type *tx_dma;
    std::array<uint8_t, TX_FIFO_BUFFER_SIZE> tx_buffer{};
    std::array<uint8_t, RX_FIFO_BUFFER_SIZE> rx_buffer{};
    uint16_t last_dma_rx_size = 0;
    State state = State::UART_STATE_READY;
    uint8_t RX_DMA_INDEX = 0;
    uint8_t RX_DMA_CHANNEL = 0;
    uint8_t TX_DMA_INDEX = 0;
    uint8_t TX_DMA_CHANNEL = 0;
};
}

#endif