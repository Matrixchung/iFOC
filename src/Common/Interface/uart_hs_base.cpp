#include "uart_hs_base.hpp"

namespace iFOC::HAL
{
void UARTHSBase::Update() {}

uint16_t UARTHSBase::WriteBytes(const uint8_t* data, uint16_t size)
{
    const uint16_t len = MIN(size, GetTxAvailable());
    tx_fifo.put(data, len);
    return len;
}

uint16_t UARTHSBase::ReadBytes(uint8_t* data, uint16_t size, const bool peek)
{
    const uint16_t len = MIN(size, GetRxLen());
    if(peek) rx_fifo.peek(data, len);
    else rx_fifo.get(data, len);
    return len;
}
}
