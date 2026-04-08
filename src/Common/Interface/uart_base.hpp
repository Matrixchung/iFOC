#pragma once

#include <cstdint>
#include <functional>
#include "../../DataType/Headers/Base/func_ret_code.h"
#include "../foc_types.hpp"
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
        OVERRIDE_NEW();
        DELETE_COPY_CONSTRUCTOR(RxEventHandlerTask);
    private:
        static constexpr size_t RX_FIFO_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384) ? 512 : 256;
        UARTBase* uart = nullptr;
        struct EventCallbackItem
        {
            uint8_t id;
            EventCallback callback;
        };
        uint8_t next_id = 0;
    public:
        Vector<EventCallbackItem> event_list{};
        std::array<uint8_t, RX_FIFO_BUFFER_SIZE> buffer{};
        explicit RxEventHandlerTask(UARTBase* _uart);
        void InitNormal() override;
        void UpdateNormal() override;
        uint8_t RegisterHandler(const EventCallback& cb);
        void RemoveHandler(uint8_t id);
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

    __fast_inline uint8_t RegisterRxHandler(const EventCallback& cb) { return event_handler.RegisterHandler(cb); };
    __fast_inline void RemoveRxHandler(const uint8_t id) { event_handler.RemoveHandler(id); };

    [[nodiscard]] __fast_inline auto GetRxLen() const { return rx_fifo.used(); };
    [[nodiscard]] __fast_inline auto GetTxPending() const { return tx_fifo.used(); };
    [[nodiscard]] __fast_inline auto GetTxAvailable() const { return tx_fifo.available(); };
};

template<typename T>
concept UARTImpl = std::is_base_of<UARTBase, T>::value;
}