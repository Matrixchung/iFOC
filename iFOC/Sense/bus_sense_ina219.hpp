#ifndef _FOC_BUS_SENSE_INA219_BASE_H
#define _FOC_BUS_SENSE_INA219_BASE_H

#include "bus_sense_base.hpp"
#include "i2c_base.h"

template <I2CImpl T>
class BusSenseINA219 : public BusSenseBase<BusSenseINA219<T>>
{
public:
    BusSenseINA219(T& _i2c, uint8_t _addr, float _max_measure_current, float _rshunt_ohm)
            : i2c(_i2c), device_addr(_addr), max_current(_max_measure_current), rshunt_ohm(_rshunt_ohm)
    {
        current_lsb = max_current / 32768.0f;
        power_lsb = current_lsb * 20.0f;
        current_cal_reg = (uint16_t)(0.04096f / (current_lsb * rshunt_ohm));
    };
    BusSenseINA219(T& _i2c, float _max_measure_current, float _rshunt_ohm)
            : BusSenseINA219(_i2c, 0x40, _max_measure_current, _rshunt_ohm) {};
    void Init();
    void Update();
private:
    T& i2c;
    uint8_t device_addr = 0x40;
    float max_current = 1.0f;
    float rshunt_ohm = 0.1f;
    uint8_t i2c_buffer[3] = {0};
    uint16_t current_cal_reg = 0;
    float current_lsb = 0.001f;
    float power_lsb = 0.025f;
};

template <I2CImpl T>
void BusSenseINA219<T>::Init()
{
    i2c.Init();
    i2c_buffer[0] = 0x00; // Config register
    // 00010 1100 1100 111b = 00010110 01100111 = 0x16 0x67
    // 16V FSR, PGA +-160mV (GAIN /4), ADC 1100 (16 Samples, 8.51ms Conv. Time), Continuous Mode
    i2c_buffer[1] = 0x16;
    i2c_buffer[2] = 0x67;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
    i2c.DelayMs(100);
    i2c_buffer[0] = 0x05; // Calibration register
    i2c_buffer[1] = (uint8_t)(current_cal_reg >> 8);
    i2c_buffer[2] = (uint8_t)current_cal_reg;
    i2c.WriteBytes(device_addr, i2c_buffer, 3);
}

template <I2CImpl T>
void BusSenseINA219<T>::Update()
{
    i2c_buffer[0] = 0x02;
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    // uint16_t temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    uint16_t temp = ((uint16_t)i2c_buffer[0] << 5) | (i2c_buffer[1] >> 3); // 12-bit
    static_cast<BusSenseBase<BusSenseINA219<T>>*>(this)->Vbus = (float)temp * 0.004f;
    i2c_buffer[0] = 0x04;
    i2c.WriteBytes(device_addr, i2c_buffer, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    // temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    int16_t i = ((int16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    static_cast<BusSenseBase<BusSenseINA219<T>>*>(this)->Ibus = (float)i * current_lsb;
}

#endif