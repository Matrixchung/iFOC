#ifndef _FOC_SPI_BASE_H
#define _FOC_SPI_BASE_H

#include <cstdint>
#include <type_traits>

class SPIBase
{
public:
    virtual void SetCS() = 0;
    virtual void ResetCS() = 0;
    virtual uint8_t WriteBytes(const uint8_t *data, uint16_t len) = 0;
    virtual uint8_t ReadBytes(uint8_t *ret, uint16_t len) = 0;
    virtual uint8_t ReadWriteBytes(const uint8_t *data, uint8_t *ret, uint16_t len) = 0;
};

template<typename T>
concept SPIImpl = std::is_base_of<SPIBase, T>::value;

#endif