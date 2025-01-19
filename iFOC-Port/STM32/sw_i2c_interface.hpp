#ifndef _SW_I2C_INTERFACE_H
#define _SW_I2C_INTERFACE_H

#include "global_include.h"
#include "delay.h"
#include "i2c_base.h"

// The SDA and SCL pin must be configured in GPIO Open Drain output first
// I2C Protocol: 7-bit address, MSB first

class SWI2C : public I2CBase
{
public:
    SWI2C(GPIO_TypeDef *_scl_port, uint32_t _scl_pin, GPIO_TypeDef *_sda_port, uint32_t _sda_pin, uint32_t _freq)
    : scl_port(_scl_port), scl_pin(_scl_pin), sda_port(_sda_port), sda_pin(_sda_pin), freq(_freq) {};
    SWI2C(GPIO_TypeDef *_scl_port, uint32_t _scl_pin, GPIO_TypeDef *_sda_port, uint32_t _sda_pin)
    : SWI2C(_scl_port, _scl_pin, _sda_port, _sda_pin, 400000) {};
    void Init() final;
    uint8_t WriteBytes(uint8_t addr, const uint8_t *data, uint16_t len) final; // return 1 if success, else 0
    uint8_t ReadBytes(uint8_t addr, uint8_t *data, uint16_t len) final;
    void DelayMs(uint32_t ms) final { delay_ms(ms); };
    uint8_t CheckDevice(uint8_t addr); // 1 if the specific address returns ACK, else 0
private:
    inline void SDAInputMode();
    inline void SDAOutputMode();
    inline void SDAOutput(uint8_t val);
    inline void SCLOutput(uint8_t val);
    inline uint8_t SDAInput();
    inline void DelayClock() const { delay_cycle(clock_cycles); };
    void Start();
    void Stop();
    uint8_t WaitAck(); // return 1 if success, 0 if failed
    void SendAck();
    void SendNAck();
    void SendByte(uint8_t data);
    uint8_t ReceiveByte();
    GPIO_TypeDef *scl_port;
    uint32_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint32_t sda_pin;
    uint32_t clock_cycles = 1;
    uint32_t freq = 400000;
};

uint8_t SWI2C::CheckDevice(uint8_t addr)
{
    addr <<= 1; // 7-bit
    addr |= 0x01; // R/W bit
    __disable_irq();
    Start();
    SendByte(addr);
    if(!WaitAck())
    {
        __enable_irq();
        return 0;
    }
    Stop();
    __enable_irq();
    return 1;
}

uint8_t SWI2C::WriteBytes(uint8_t addr, const uint8_t *data, uint16_t len)
{
    addr <<= 1;
    addr &= ~0x01; // R/W bit
    __disable_irq();
    Start();
    SendByte(addr);
    if(!WaitAck())
    {
        __enable_irq();
        return 0;
    }
    for(uint16_t i = 0; i < len; i++)
    {
        SendByte(*(data + i));
        if(!WaitAck())
        {
            __enable_irq();
            return 0;
        }
    }
    Stop();
    __enable_irq();
    return 1;
}

uint8_t SWI2C::ReadBytes(uint8_t addr, uint8_t *data, uint16_t len)
{
    addr <<= 1;
    addr |= 0x01; // R/W bit
    __disable_irq();
    Start();
    SendByte(addr);
    if(!WaitAck())
    {
        __enable_irq();
        return 0;
    }
    for(uint16_t i = 0; i < len - 1; i++)
    {
        *(data + i) = ReceiveByte();
        SendAck();
    }
    *(data + len - 1) = ReceiveByte();
    SendNAck();
    Stop();
    __enable_irq();
    return 1;
}

void SWI2C::Init()
{
    LL_GPIO_SetPinMode(scl_port, scl_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(sda_port, sda_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(scl_port, scl_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(sda_port, sda_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(scl_port, scl_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(sda_port, sda_pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetOutputPin(scl_port, scl_pin);
    LL_GPIO_SetOutputPin(sda_port, sda_pin);
    clock_cycles = SystemCoreClock / freq;
    clock_cycles /= 2;
}

inline void SWI2C::SDAInputMode()
{
    LL_GPIO_SetPinMode(sda_port, sda_pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(sda_port, sda_pin, LL_GPIO_PULL_NO);
}

inline void SWI2C::SDAOutputMode()
{
    LL_GPIO_SetPinMode(sda_port, sda_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(sda_port, sda_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(sda_port, sda_pin, LL_GPIO_PULL_NO);
}

inline void SWI2C::SDAOutput(uint8_t val)
{
    if(val) LL_GPIO_SetOutputPin(sda_port, sda_pin);
    else LL_GPIO_ResetOutputPin(sda_port, sda_pin);
}

inline void SWI2C::SCLOutput(uint8_t val)
{
    if(val) LL_GPIO_SetOutputPin(scl_port, scl_pin);
    else LL_GPIO_ResetOutputPin(scl_port, scl_pin);
}

inline uint8_t SWI2C::SDAInput()
{
    return (uint8_t)LL_GPIO_IsInputPinSet(sda_port, sda_pin);
}

void SWI2C::Start()
{
    SDAOutput(1);
    // DelayClock();
    SCLOutput(1);
    DelayClock();
    SDAOutput(0);
    DelayClock();
    SCLOutput(0);
    DelayClock();
}

void SWI2C::Stop()
{
    SCLOutput(0);
    // DelayClock();
    SDAOutput(0);
    DelayClock();
    SCLOutput(1);
    DelayClock();
    SDAOutput(1);
    DelayClock();
}

uint8_t SWI2C::WaitAck()
{
    uint8_t timeout = 5;
    SDAInputMode();
    // DelayClock();
    SCLOutput(1);
    DelayClock();
    while(SDAInput())
    {
        timeout--;
        DelayClock();
        if(timeout == 0)
        {
            SDAOutputMode();
            Stop();
            return 0;
        }
    }
    SCLOutput(0);
    SDAOutputMode();
    // DelayClock();
    return 1;
}

void SWI2C::SendAck()
{
    SDAOutput(0);
    DelayClock();
    SCLOutput(1);
    DelayClock();
    SCLOutput(0);
    DelayClock();
}

void SWI2C::SendNAck()
{
    SDAOutput(1);
    DelayClock();
    SCLOutput(1);
    DelayClock();
    SCLOutput(0);
    DelayClock();
}

void SWI2C::SendByte(uint8_t data)
{
    uint8_t i = 8;
    while(i--)
    {
        SCLOutput(0);
        SDAOutput(data & 0x80);
        DelayClock();
        data <<= 1;
        SCLOutput(1);
        DelayClock();
    }
    SCLOutput(0);
    DelayClock();
}

uint8_t SWI2C::ReceiveByte()
{
    uint8_t i = 8;
    uint8_t ret = 0;
    SDAInputMode();
    while(i--)
    {
        ret <<= 1;
        SCLOutput(0);
        DelayClock();
        SCLOutput(1);
        DelayClock();
        ret |= SDAInput();
    }
    SCLOutput(0);
    DelayClock();
    SDAOutputMode();
    return ret;
}

#endif