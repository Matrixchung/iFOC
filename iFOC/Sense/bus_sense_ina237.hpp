#ifndef _FOC_BUS_SENSE_INA226_BASE_H
#define _FOC_BUS_SENSE_INA226_BASE_H

#include "bus_sense_base.hpp"
#include "i2c_base.h"

// https://www.ti.com/cn/lit/ds/symlink/ina237.pdf

template <I2CImpl T>
class BusSenseINA237 : public BusSenseBase<BusSenseINA237<T>>
{
public:
    BusSenseINA237(T& _i2c, uint8_t _addr, float _max_measure_current, float _rshunt_ohm)
    : i2c(_i2c), device_addr(_addr), max_current(_max_measure_current), rshunt_ohm(_rshunt_ohm)
    {
        current_lsb = max_current / 32768.0f;
        power_lsb = current_lsb * 0.2f;
        current_cal_reg = (uint16_t)(819200000.0f * current_lsb * rshunt_ohm);
    };
    BusSenseINA237(T& _i2c, float _max_measure_current, float _rshunt_ohm)
    : BusSenseINA237(_i2c, 0x40, _max_measure_current, _rshunt_ohm) {};
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

template <I2CImpl T>
void BusSenseINA237<T>::Init()
{
    i2c.Init();
    // i2c_buffer[0] = 0x00; // Config register, default ADCRANGE = +-163.84mV
    // i2c_buffer[1] = 0x45;
    // i2c_buffer[2] = 0x27;
    // i2c.WriteBytes(device_addr, i2c_buffer, 3);
    i2c_buffer[0] = 0x01; // ADC_CONFIG Register, enabling AVG=1
    i2c_buffer[1] = 0xFB;
    i2c_buffer[2] = 0x69;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
    i2c.DelayMs(100);
    i2c_buffer[0] = 0x02; // Calibration register
    i2c_buffer[1] = (uint8_t)(current_cal_reg >> 8);
    i2c_buffer[2] = (uint8_t)current_cal_reg;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
}

template <I2CImpl T>
void BusSenseINA237<T>::Update()
{
    i2c_buffer[0] = 0x05; // Bus Voltage
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    uint16_t temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    static_cast<BusSenseBase<BusSenseINA237<T>>*>(this)->Vbus = (float)temp * 0.003125f;
    i2c_buffer[0] = 0x07; // Current Result
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    static_cast<BusSenseBase<BusSenseINA237<T>>*>(this)->Ibus = (float)temp * current_lsb;
}

#endif