#ifndef _FOC_BUS_SENSE_INA226_BASE_H
#define _FOC_BUS_SENSE_INA226_BASE_H

#include "bus_sense_base.hpp"

// https://blog.csdn.net/weixin_50257954/article/details/133635534

template <typename T>
class BusSenseINA226Base : public BusSenseBase<BusSenseINA226Base>
{
public:
    BusSenseINA226Base(uint8_t _addr, float _max_measure_current, float _rshunt_ohm)
    : device_addr(_addr), max_current(_max_measure_current), rshunt_ohm(_rshunt_ohm)
    {
        current_lsb = max_current / 32768.0f;
        power_lsb = current_lsb * 25.0f;
    };
    void Init();
    void Update();
private:
    uint8_t device_addr = 0x80;
    float max_current = 10.0f;
    float rshunt_ohm = 0.002f;
    uint16_t i2c_buffer = 0;
    float current_lsb = 0.001f;
    float power_lsb = 0.025f;
    inline void I2CInit() { static_cast<T*>(this)->PortI2CInit(); }
    inline void I2CWriteBytes(const uint8_t *data, uint8_t len) { static_cast<T*>(this)->PortI2CWriteBytes(device_addr, data, len); };
    inline void I2CReadBytes(uint8_t *data, uint8_t len) { static_cast<T*>(this)->PortI2CReadBytes(device_addr, data, len); };
};

template <typename T>
void BusSenseINA226Base<T>::Init()
{
    I2CInit();

}

#endif