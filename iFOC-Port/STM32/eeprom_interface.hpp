#ifndef _FOC_EEPROM_INTERFACE_H
#define _FOC_EEPROM_INTERFACE_H

#include "foc_header.h"
#include "foc_config_type.h"
#include "i2c_base.h"
#include "string.h"

// AT24C256C: 262144bits = 32768Bytes = 8192floats

// use union to extract/save multiple-byte variables
// For EEPROM:
// At init, we will read all packet based on given sud_dev_index, and check crc.
// If CRC correct, then we will apply those settings to FOC config struct.
// If CRC WRONG, we will writeback all default settings to EEPROM here.

template<class T = I2CBase>
class EEPROM
{
public:
    EEPROM(T& _i2c, uint8_t _addr) : i2c(_i2c), device_addr(_addr)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
    };
    EEPROM(T& _i2c) : EEPROM(_i2c, 0x50) {};
    template<class U>
    [[nodiscard]] U ReadVar(uint16_t addr);
    template<class U>
    void WriteVar(uint16_t addr, U var);
    void FlushPage(uint16_t addr);
    void SaveConfigBlob(const char *key, const nvm_config_t *blob);
    bool ReadConfigBlob(const char *key, nvm_config_t *blob);
private:
    T& i2c;
    void PageWrite(uint16_t start_addr, uint8_t *src, uint16_t len);
    void SequentialRead(uint16_t start_addr, uint8_t *dst, uint16_t len);
    uint8_t device_addr = 0x50; // AT24C256C: 0x50
    uint8_t buffer[32];
};

template<class T>
template<class U>
U EEPROM<T>::ReadVar(uint16_t addr)
{
    static_assert(sizeof(U) < 32, "Length of required data type exceeds limit");
    union
    {
        uint8_t u8[sizeof(U)];
        U dest;
    };
    SequentialRead(addr, u8, sizeof(U));
    return dest;
}

template<class T>
template<class U>
void EEPROM<T>::WriteVar(uint16_t addr, U var)
{
    static_assert(sizeof(U) < 32, "Length of required data type exceeds limit");
    union
    {
        uint8_t u8[sizeof(U)];
        U dest;
    };
    dest = var;
    PageWrite(addr, u8, sizeof(U));
}

template<class T>
void EEPROM<T>::FlushPage(uint16_t addr)
{
    uint8_t dummy[31] = {0};
    PageWrite(addr, dummy, 31);
}

template<class T>
void EEPROM<T>::PageWrite(uint16_t start_addr, uint8_t *src, uint16_t len)
{
    buffer[0] = (uint8_t)(start_addr >> 8);
    buffer[1] = (uint8_t)start_addr;
    memcpy(buffer + 2, src, len);
    i2c.WriteBytes(device_addr, buffer, len + 2);
}

template<class T>
void EEPROM<T>::SequentialRead(uint16_t start_addr, uint8_t *src, uint16_t len)
{
    buffer[0] = (uint8_t)(start_addr >> 8);
    buffer[1] = (uint8_t)start_addr;
    i2c.WriteBytes(device_addr, buffer, 2);
    i2c.ReadBytes(device_addr, src, len);
}

template<class T>
void EEPROM<T>::SaveConfigBlob(const char *key, const nvm_config_t *blob)
{
    auto parsed_idx = key[strlen(key) - 1];
    if(parsed_idx >= '0' && parsed_idx <= '9')
    {
        auto sub_dev_index = (uint8_t)(parsed_idx - '0');
        WriteVar<nvm_config_t>(sub_dev_index * sizeof(nvm_config_t), *blob);
    }
}

template<class T>
bool EEPROM<T>::ReadConfigBlob(const char *key, nvm_config_t *blob)
{
    auto parsed_idx = key[strlen(key) - 1];
    if(parsed_idx >= '0' && parsed_idx <= '9')
    {
        auto sub_dev_index = (uint8_t)(parsed_idx - '0');
        *blob = ReadVar<nvm_config_t>(sub_dev_index * sizeof(nvm_config_t));
        return true;
    }
    return false;
}

#endif