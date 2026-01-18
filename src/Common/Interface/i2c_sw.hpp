#pragma once

#include "gpio_base.hpp"
#include "i2c_base.hpp"
#include "../foc_types.hpp"

namespace iFOC::HAL
{
class I2CSW final : public I2CBase
{
public:
    I2CSW(GPIOBase& _scl, GPIOBase& _sda);
    I2CSW(GPIOBase& _scl, GPIOBase& _sda, uint32_t _freq);
    I2CSW(GPIOBase* _scl, GPIOBase* _sda);
    I2CSW(GPIOBase* _scl, GPIOBase* _sda, uint32_t _freq);
    ~I2CSW();
    FuncRetCode Init() final;
    FuncRetCode WriteBytes(uint8_t addr, const uint8_t* data, uint16_t size) final;
    FuncRetCode ReadBytes(uint8_t addr, uint8_t* data, uint16_t size) final;
private:
    inline void I2CStart() const;
    inline void I2CStop() const;
    inline bool WaitAck() const;
    inline void SendAck() const;
    inline void SendNAck() const;
    inline void SendByte(uint8_t data) const;
    inline uint8_t ReceiveByte() const;
    inline void _delay() const;
    GPIOBase& scl;
    GPIOBase& sda;
    uint32_t clock_cycles = 1;
    uint32_t freq = 400000;
};
}