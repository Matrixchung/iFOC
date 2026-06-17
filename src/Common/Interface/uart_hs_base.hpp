#pragma once

#include "../../DataType/Headers/Base/func_ret_code.h"
#include "../foc_types.hpp"
#include "../../DataType/Headers/Comm/uart_baudrate.h"
#include "../../DataType/Ringbuf/kfifo.hpp"

namespace iFOC::HAL
{
/// UART High-Speed(HS) interface under polling mode, only one consumer is allowed. \n
/// Consumer task should poll the Update() function periodically,
/// and handle the hardware flags without any IRQ used. \n
/// For Producer-Consumer mode interface, see UART interface in uart_base.hpp.
class UARTHSBase
{
    DELETE_COPY_CONSTRUCTOR(UARTHSBase);
    OVERRIDE_NEW();
// protected:
//     DataType::Ringbuf::kfifo_t tx_fifo;
//     DataType::Ringbuf::kfifo_t rx_fifo;
public:
    UARTHSBase() = default;
    virtual ~UARTHSBase() = default;

    /// Initialize the UART HS interface.
    /// \param baud UARTBaudrate, fallback to 115200 if argument invalid
    /// \return FuncRetCode
    virtual FuncRetCode Init(DataType::Comm::UARTBaudrate baud) = 0;

    /// By polling this function periodically, a consumer should have
    /// updated incoming bytes in the Rx FIFO. Then the handling process is
    /// dominated by ReadBytes().
    virtual void Update();

    /// Start transmit after multiple WriteBytes(), forced by DMA.
    virtual void StartTransmit() = 0;

    /// Write a specific amount of data to the Tx FIFO.
    /// \param data pointer to src array
    /// \param size size of data requested to be written
    /// \return actual size wrote into the Tx FIFO
    uint16_t WriteBytes(const uint8_t* data, uint16_t size);

    /// Read a specific amount of data from the Rx FIFO.
    /// \param data pointer to dst array
    /// \param size size of data requested to be read
    /// \param peek if true, fifo data won't be deleted
    /// \return actual size read from the Rx FIFO
    uint16_t ReadBytes(uint8_t* data, uint16_t size, bool peek);

    [[nodiscard]] __fast_inline auto GetRxLen() const { return rx_fifo.used(); };
    [[nodiscard]] __fast_inline auto GetTxPending() const { return tx_fifo.used(); };
    [[nodiscard]] __fast_inline auto GetTxAvailable() const { return tx_fifo.available(); };

    DataType::Ringbuf::kfifo_t tx_fifo;
    DataType::Ringbuf::kfifo_t rx_fifo;
};
}