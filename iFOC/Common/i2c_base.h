#ifndef _FOC_I2C_BASE_H
#define _FOC_I2C_BASE_H

#include "stdint.h"

class I2CBase
{
public:
    virtual void Init() = 0;
    virtual uint8_t WriteBytes(uint8_t addr, uint8_t *data, uint8_t len) = 0; // return 1 if success, else 0
    virtual uint8_t ReadBytes(uint8_t addr, uint8_t *data, uint8_t len) = 0;
    virtual void DelayMs(uint32_t ms) = 0;
};

#endif