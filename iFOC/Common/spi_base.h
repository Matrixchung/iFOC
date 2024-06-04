#ifndef _FOC_SPI_BASE_H
#define _FOC_SPI_BASE_H

#include "stdint.h"

class SPIBase
{
public:
    virtual void Init() = 0;
    virtual void SetCS(uint8_t state) = 0;
    virtual void ReadWriteBytes(uint8_t *data, uint8_t len) = 0;
};

#endif