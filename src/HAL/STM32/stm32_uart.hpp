#pragma once

#include "uart_base.hpp"
#include "main.h"

#if defined(HAL_UART_MODULE_ENABLED)

namespace iFOC::HAL
{
/// Required Settings: 1) USART DMA Tx/Rx Interrupt AND global interrupt enabled, with a priority number
///                       higher than LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5). \n
///                    2) USART DMA Rx Circular Mode, Tx Normal Mode. (Alignment: BYTE) \n
///                    3) USART Overrun Flag(ORE) Disabled
class STM32UART final : public UARTBase
{
public:
    explicit STM32UART(UART_HandleTypeDef *_huart);
    FuncRetCode Init(DataType::Comm::UARTBaudrate baud) final;
    FuncRetCode StartTransmit(bool blocked) final;
    void OnIdleIRQ(UART_HandleTypeDef *_huart, uint16_t Size);
    void OnTxCpltIRQ(UART_HandleTypeDef *_huart);
private:
    static constexpr size_t TX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
    static constexpr size_t HALF_RX_FIFO_BUFFER_SIZE = RX_FIFO_BUFFER_SIZE / 2;
    UART_HandleTypeDef *huart;
    std::array<uint8_t, TX_FIFO_BUFFER_SIZE> tx_buffer;
    std::array<uint8_t, RX_FIFO_BUFFER_SIZE> rx_buffer;
    uint16_t last_dma_rx_size = 0;
};
}

#endif