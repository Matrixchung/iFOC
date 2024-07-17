#ifndef _FOC_SPI_BASE_H
#define _FOC_SPI_BASE_H

#include "stdint.h"

class SPIBase
{
public:
    virtual void SetCS() = 0;
    virtual void ResetCS() = 0;
    virtual uint8_t WriteBytes(uint8_t *data, uint16_t len) = 0;
    virtual uint8_t ReadBytes(uint8_t *ret, uint16_t len) = 0;
    virtual uint8_t ReadWriteBytes(uint8_t *data, uint8_t *ret, uint16_t len) = 0;
};

#endif