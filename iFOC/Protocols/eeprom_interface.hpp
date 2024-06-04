#ifndef _FOC_EEPROM_INTERFACE_H
#define _FOC_EEPROM_INTERFACE_H

#include "foc_header.h"
#include "i2c_base.h"

typedef enum EEPROM_ADDR_OFFSET
{
    NODE_ID = 0,
    
    CRC_BIT, // last, will save CRC8 calculated value from all above(except itself)
}EEPROM_ADDR_OFFSET;

template<typename T = I2CBase>
class EEPROM
{
public:
    EEPROM(T& _i2c, uint8_t _addr) : i2c(_i2c), device_addr(_addr)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
    };
    T& i2c;
private:
    uint8_t device_addr = 0x00;
};

#endif