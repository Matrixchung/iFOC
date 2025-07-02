#pragma once

#include <cstdint>
#include <functional>
#include "func_ret_code.h"
#include "foc_types.hpp"
#include "../../DataType/Headers/Comm/uart_baudrate.h"
#include "../../DataType/Ringbuf/kfifo.hpp"
#include "../foc_task.hpp"
#include "semphr.h"

namespace iFOC::HAL
{
class UARTBase
{
    DELETE_COPY_CONSTRUCTOR(UARTBase);
    OVERRIDE_NEW();
public:
    // if false: next registered event can continue process. true means message has been successfully handled by current callback.
    using EventCallback = std::function<bool(uint8_t*, uint16_t)>;
protected:
    static constexpr TickType_t READ_WRITE_TIMEOUT_MS = 100;
    class RxEventHandlerTask final : public Task
    {
    private:
        UARTBase* uart;
    public:
        Vector<EventCallback> event_list;
        std::array<uint8_t, 256> buffer;
        explicit RxEventHandlerTask(UARTBase* _uart);
        void InitNormal() final;
        void UpdateNormal() final;
        void RegisterHandler(EventCallback cb);
    };
    friend class RxEventHandlerTask;
    RxEventHandlerTask event_handler;
    DataType::Ringbuf::kfifo_t tx_fifo;
    DataType::Ringbuf::kfifo_t rx_fifo;
    SemaphoreHandle_t tx_sem = nullptr;
    // SemaphoreHandle_t rx_sem = nullptr; // using Task Notification
    /// FIFO Mutex for multi-producer, multi-consumer application.
    SemaphoreHandle_t tx_fifo_mutex = nullptr;
    // SemaphoreHandle_t rx_fifo_mutex = nullptr;
    // __fast_inline void CallTxCpltCallback() { EXECUTE(tx_cplt_cb); };
    // __fast_inline void CallRxCpltCallback() { EXECUTE(rx_cplt_cb); };
    // __fast_inline void WaitUntilRXNE(TickType_t delay = portMAX_DELAY) { xSemaphoreTake(rx_sem, delay); };
public:
    UARTBase();
    virtual ~UARTBase();

    /// Initialize the UART interface. \n
    /// xSemaphoreGive(tx_sem); and RxEventHandlerTask must be started.
    /// \param baud UARTBaudrate, fallback to 115200 if argument invalid
    /// \return FuncRetCode
    virtual FuncRetCode Init(DataType::Comm::UARTBaudrate baud) = 0;

    /// Write a specific amount of data to the FIFO.
    /// \param data pointer to src array
    /// \param size size of data requested to be wrote
    /// \return actual size wrote into the FIFO
    virtual uint16_t WriteBytes(const uint8_t* data, uint16_t size);

    /// Read a specific amount of data from the received FIFO.
    /// \param data pointer to dst array
    /// \param size size of data requested to be read
    /// \param peek if true, fifo data won't be deleted
    /// \return actual size read from the FIFO
    virtual uint16_t ReadBytes(uint8_t* data, uint16_t size, bool peek);

    /// Start transmit after multiple WriteBytes()
    /// \param blocked whether use blocking transmission or DMA
    /// \return FuncRetCode
    virtual FuncRetCode StartTransmit(bool blocked) = 0;

    void Print(bool transmit, const char *fmt, ...);

    __fast_inline void RegisterRxHandler(EventCallback cb) { event_handler.RegisterHandler(cb); };

    __fast_inline auto GetRxLen() { return rx_fifo.used(); };
    __fast_inline auto GetTxPending() { return tx_fifo.used(); };
    __fast_inline auto GetTxAvailable() { return tx_fifo.available(); };
};
}