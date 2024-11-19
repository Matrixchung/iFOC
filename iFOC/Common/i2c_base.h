#ifndef _FOC_I2C_BASE_H
#define _FOC_I2C_BASE_H

#include <cstdint>
#include <type_traits>

class I2CBase
{
public:
    virtual void Init() = 0;
    virtual uint8_t WriteBytes(uint8_t addr, uint8_t *data, uint16_t len) = 0; // return 1 if success, else 0
    virtual uint8_t ReadBytes(uint8_t addr, uint8_t *data, uint16_t len) = 0;
    virtual void DelayMs(uint32_t ms) = 0;
};

template<typename T>
concept I2CImpl = std::is_base_of<I2CBase, T>::value;

#endif