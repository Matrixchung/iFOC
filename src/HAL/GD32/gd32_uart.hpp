#pragma once

#include "../../Common/Interface/uart_base.hpp"
#include "hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::HAL
{
class UART final : public UARTBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(UART);
public:
    UART(uint32_t _huart, uint32_t _rx_dma_inst, dma_channel_enum _rx_dma_ch, uint32_t _tx_dma_inst, dma_channel_enum _tx_dma_ch);
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
    FuncRetCode TransmitBlocking(const uint8_t* data, uint16_t size, TickType_t timeout);
    FuncRetCode TransmitDMA(const uint8_t* data, uint16_t size);
    static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    uint32_t huart;
    uint32_t rx_dma_inst;
    dma_channel_enum rx_dma_ch;
    uint32_t tx_dma_inst;
    dma_channel_enum tx_dma_ch;
    std::array<uint8_t, TX_FIFO_BUFFER_SIZE> tx_buffer{};
    std::array<uint8_t, RX_FIFO_BUFFER_SIZE> rx_buffer{};
    uint16_t last_dma_rx_size = 0;
    State state = State::UART_STATE_READY;
};
}

#endif