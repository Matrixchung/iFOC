#ifndef _FOC_BUS_SENSE_INA226_BASE_H
#define _FOC_BUS_SENSE_INA226_BASE_H

#include "bus_sense_base.hpp"
#include "i2c_base.h"

// https://blog.csdn.net/weixin_50257954/article/details/133635534

template <typename T = I2CBase>
class BusSenseINA226 : public BusSenseBase<BusSenseINA226<T>>
{
public:
    BusSenseINA226(T& _i2c, uint8_t _addr, float _max_measure_current, float _rshunt_ohm)
    : i2c(_i2c), device_addr(_addr), max_current(_max_measure_current), rshunt_ohm(_rshunt_ohm)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
        current_lsb = max_current / 32768.0f;
        power_lsb = current_lsb * 25.0f;
        current_cal_reg = (uint16_t)(0.00512f / (current_lsb * rshunt_ohm));
    };
    BusSenseINA226(T& _i2c, float _max_measure_current, float _rshunt_ohm)
    : i2c(_i2c), max_current(_max_measure_current), rshunt_ohm(_rshunt_ohm)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
        current_lsb = max_current / 32768.0f;
        power_lsb = current_lsb * 25.0f;
        current_cal_reg = (uint16_t)(0.00512f / (current_lsb * rshunt_ohm));
    };
    void Init();
    void Update();
private:
    T& i2c;
    uint8_t device_addr = 0x40;
    float max_current = 10.0f;
    float rshunt_ohm = 0.002f;
    uint8_t i2c_buffer[3] = {0};
    uint16_t current_cal_reg = 0;
    float current_lsb = 0.001f;
    float power_lsb = 0.025f;
};

template <typename T>
void BusSenseINA226<T>::Init()
{
    i2c.Init();
    i2c_buffer[0] = 0x00; // Config register
    i2c_buffer[1] = 0x45;
    i2c_buffer[2] = 0x27;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
    i2c.DelayMs(100);
    i2c_buffer[0] = 0x05; // Calibration register
    i2c_buffer[1] = (uint8_t)(current_cal_reg >> 8);
    i2c_buffer[2] = (uint8_t)current_cal_reg;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
}

template <typename T>
void BusSenseINA226<T>::Update()
{
    i2c_buffer[0] = 0x02;
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    uint16_t temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    static_cast<BusSenseBase<BusSenseINA226<T>>*>(this)->Vbus = (float)temp * 0.00125f;
    i2c_buffer[0] = 0x04;
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    static_cast<BusSenseBase<BusSenseINA226<T>>*>(this)->Ibus = (float)temp * current_lsb;
}

#endif