#include "uart_base.hpp"
#include "../../Protocol/ascii_tiny_printf.hpp"

namespace iFOC::HAL
{
UARTBase::UARTBase() : event_handler(this)
{
    tx_fifo_mutex = xSemaphoreCreateMutex();
    // rx_fifo_mutex = xSemaphoreCreateMutex();
    tx_sem = xSemaphoreCreateBinary();
    // rx_sem = xSemaphoreCreateBinary();
}

UARTBase::~UARTBase()
{
    vSemaphoreDelete(tx_fifo_mutex);
    // vSemaphoreDelete(rx_fifo_mutex);
    vSemaphoreDelete(tx_sem);
    // vSemaphoreDelete(rx_sem);
}

static constexpr size_t PRINT_BUFFER_SIZE = 128;
void UARTBase::Print(bool transmit, const char *fmt, ...)
{
    char buffer[PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    auto len = (uint16_t)vsnprintf_(buffer, sizeof(buffer), fmt, args);
    if(GetTxAvailable() < len) StartTransmit(false); // FIFO used up, we have to initiate a transmission
    WriteBytes((const uint8_t*)buffer, len);
    if(transmit) StartTransmit(false);
    va_end(args);
}

uint16_t UARTBase::WriteBytes(const uint8_t *data, uint16_t size)
{
    if(xSemaphoreTake(tx_fifo_mutex, pdMS_TO_TICKS(READ_WRITE_TIMEOUT_MS)) == pdTRUE)
    {
        uint16_t len = MIN(size, GetTxAvailable());
        tx_fifo.put(data, len);
        xSemaphoreGive(tx_fifo_mutex);
        return len;
    }
    return 0;
}

uint16_t UARTBase::ReadBytes(uint8_t *data, uint16_t size, bool peek)
{
    // xSemaphoreTake(rx_fifo_mutex, portMAX_DELAY);
    uint16_t len = MIN(size, GetRxLen());
    if(peek) rx_fifo.peek(data, len);
    else rx_fifo.get(data, len);
    // xSemaphoreGive(rx_fifo_mutex);
    return len;
}

UARTBase::RxEventHandlerTask::RxEventHandlerTask(UARTBase* _uart) : Task("UARTHandler"), uart(_uart)
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 1; // IRQ-Called Task, maximum priority.
    config.stack_depth = 512;
}

void UARTBase::RxEventHandlerTask::InitNormal()
{
    event_list.reserve(4);
}

void UARTBase::RxEventHandlerTask::UpdateNormal()
{
    // https://forums.freertos.org/t/task-stuck-at-tasknotification-received/6113/15
    if(ulTaskNotifyTake(pdTRUE, 10) == pdTRUE)
    {
        iFOC::TaskTimer timer;
        MEASURE_TIME(timer)
        {
            auto len = uart->ReadBytes(buffer.data(), buffer.max_size(), false);
            if(len > 0) for(const auto& cb : event_list) if(cb(buffer.data(), len)) break;
        }
        uart->Print(1, "Time:%d\n", timer.elapsed_time_us);
    }
}

void UARTBase::RxEventHandlerTask::RegisterHandler(UARTBase::EventCallback cb)
{
    event_list.push_back(cb);
}

}
