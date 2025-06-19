#include "i2c_sw.hpp"
#include "foc_math.hpp"

namespace iFOC::HAL
{
I2CSW::I2CSW(GPIOBase &_scl, GPIOBase &_sda) : scl(_scl), sda(_sda) {};

I2CSW::I2CSW(GPIOBase &_scl, GPIOBase &_sda, uint32_t _freq) : scl(_scl), sda(_sda), freq(_freq)
{
    if(!BETWEEN(freq, 50000, 10000000)) misconfigured_area |= to_underlying(MisconfiguredArea::I2C_FREQ_OUT_OF_RANGE);
}

I2CSW::I2CSW(GPIOBase *_scl, GPIOBase *_sda) : I2CSW(*_scl, *_sda) {};
I2CSW::I2CSW(GPIOBase *_scl, GPIOBase *_sda, uint32_t _freq) : I2CSW(*_scl, *_sda, _freq) {};

I2CSW::~I2CSW()
{
    scl.ModeOutOD();
    sda.ModeOutOD();
}

FuncRetCode I2CSW::WriteBytes(uint8_t addr, const uint8_t* data, const uint16_t size)
{
    addr <<= 1;
    addr &= ~0x01;
    // DisableIRQ();
    portENTER_CRITICAL();
    I2CStart();
    SendByte(addr);
    if(!WaitAck()) goto timeout;
    // for(const auto data : sp)
    // {
    //     SendByte(data);
    //     if(!WaitAck()) goto timeout;
    // }
    for(auto i = 0; i < size; i++)
    {
        SendByte(*(data + i));
        if(!WaitAck()) goto timeout;
    }
    I2CStop();
    // EnableIRQ();
    portEXIT_CRITICAL();
    return FuncRetCode::OK;
    timeout:
    // EnableIRQ();
    portEXIT_CRITICAL();
    return FuncRetCode::REMOTE_TIMEOUT;
}

FuncRetCode I2CSW::ReadBytes(uint8_t addr, uint8_t* data, const uint16_t size)
{
    // if(!sp.size()) return FuncRetCode::PARAM_OUT_BOUND; // Disable zero length check
    // std::span<uint8_t> sub_sp = sp.first(sp.size() - 1);
    addr <<= 1;
    addr |= 0x01;
    // DisableIRQ();
    portENTER_CRITICAL();
    I2CStart();
    SendByte(addr);
    if(!WaitAck()) goto timeout;
    // for(auto & data : sub_sp)
    // {
    //     data = ReceiveByte();
    //     SendAck();
    // }
    // sp.back() = ReceiveByte();
    // for(auto it = sp.begin(); it != sp.end() - 1; ++it)
    // {
    //     *it = ReceiveByte();
    //     SendAck();
    // }
    // sp.back() = ReceiveByte();
    for(auto i = 0; i < size - 1; i++)
    {
        *(data + i) = ReceiveByte();
        SendAck();
    }
    *(data + size - 1) = ReceiveByte();
    SendNAck();
    I2CStop();
    // EnableIRQ();
    portEXIT_CRITICAL();
    return FuncRetCode::OK;
    timeout:
    // EnableIRQ();
    portEXIT_CRITICAL();
    return FuncRetCode::REMOTE_TIMEOUT;
}

FuncRetCode I2CSW::Init()
{
    scl.ModeOutOD();
    scl = 1;
    sda.ModeOutOD();
    sda = 1;
    scl.PullNo();
    sda.PullNo();
    clock_cycles = iFOC::HAL::GetCoreClockHz() / freq;
    clock_cycles /= 2;
    return FuncRetCode::OK;
}

void I2CSW::I2CStart()
{
    sda = 1;
    scl = 1;
    _delay();
    sda = 0;
    _delay();
    scl = 0;
    _delay();
}

void I2CSW::I2CStop()
{
    scl = 0;
    sda = 0;
    _delay();
    scl = 1;
    _delay();
    sda = 1;
    _delay();
}

bool I2CSW::WaitAck()
{
    uint8_t timeout = 5;
    sda.ModeInput();
    scl = 1;
    _delay();
    while(sda)
    {
        timeout--;
        _delay();
        if(!timeout)
        {
            sda.ModeOutOD();
            I2CStop();
            return false;
        }
    }
    scl = 0;
    sda.ModeOutOD();
    return true;
}

void I2CSW::SendAck()
{
    sda = 0;
    _delay();
    scl = 1;
    _delay();
    scl = 0;
    _delay();
}

void I2CSW::SendNAck()
{
    sda = 1;
    _delay();
    scl = 1;
    _delay();
    scl = 0;
    _delay();
}

void I2CSW::SendByte(uint8_t data)
{
    uint8_t i = 8;
    while(i--)
    {
        scl = 0;
        sda = data & 0x80;
        _delay();
        data <<= 1;
        scl = 1;
        _delay();
    }
    scl = 0;
    _delay();
}

uint8_t I2CSW::ReceiveByte()
{
    uint8_t i = 8, ret = 0;
    sda.ModeInput();
    while(i--)
    {
        ret <<= 1;
        scl = 0;
        _delay();
        scl = 1;
        _delay();
        ret |= sda.Read();
    }
    scl = 0;
    _delay();
    sda.ModeOutOD();
    return ret;
}

void I2CSW::_delay() const
{
    HAL::DelayCycle(clock_cycles);
}

}