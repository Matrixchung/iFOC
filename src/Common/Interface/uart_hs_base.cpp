#include "uart_hs_base.hpp"

namespace iFOC::HAL
{
void UARTHSBase::UpdateRxFIFO() {}

void UARTHSBase::RegisterIdleCallback(IdleCallback cb)
{
    idle_cb = std::move(cb);
}

void UARTHSBase::RemoveIdleCallback()
{
    idle_cb = nullptr;
}

uint16_t UARTHSBase::WriteBytes(const uint8_t* data, const uint16_t size)
{
    // const uint16_t len = MIN(size, GetTxAvailable());
    const uint16_t len = size;
    tx_fifo.put(data, len);
    return len;
}

uint16_t UARTHSBase::ReadBytes(uint8_t* data, const uint16_t size, const bool peek)
{
    // const uint16_t len = MIN(size, GetRxLen());
    const uint16_t len = size;
    if(peek) rx_fifo.peek(data, len);
    else rx_fifo.get(data, len);
    return len;
}
}
