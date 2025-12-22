#include "bus_sense_ina237.hpp"

#define config iFOC::BoardConfig().GetConfig()

namespace iFOC::Sense
{
BusSenseINA237::BusSenseINA237(I2CBase *_i2c, uint8_t _addr, float _max, float _shunt, bool rev) :
        i2c(_i2c), device_addr(_addr), max_current(_max), rshunt_ohm(_shunt), current_lsb(_max / 32768.0f), reversed(rev)
{
    // power_lsb = current_lsb * 0.2f;
    current_cal_reg = (uint16_t)(819200000.0f * current_lsb * rshunt_ohm);
}

BusSenseINA237::BusSenseINA237(I2CBase *_i2c) :
        BusSenseINA237(_i2c,
                       0x40,
                       config.bus_max_positive_current() * 2.0f,
                       config.bus_sense_shunt_ohm(), false) {}

BusSenseINA237::BusSenseINA237(I2CBase* _i2c, uint8_t _addr, bool rev) :
        BusSenseINA237(_i2c,
                       _addr,
                       config.bus_max_positive_current() * 2.0f,
                       config.bus_sense_shunt_ohm(), rev) {}

FuncRetCode BusSenseINA237::Init()
{
    if (!BETWEEN(max_current, 0.0f, 1000.0f) ||
        !BETWEEN(rshunt_ohm, 0.00001f, 1.0f))
    {
        return FuncRetCode::PARAM_OUT_BOUND;
    }
    i2c->Init();
    /// Step #1: Read MANUFACTURER_ID register, it should be 0x5449
    i2c_buffer[0] = 0x3E;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    if(i2c_buffer[0] != 0x54 || i2c_buffer[1] != 0x49)
    {
        if(i2c_buffer[0] != 0x00 && i2c_buffer[1] != 0x00) return FuncRetCode::CRC_MISMATCH;
        return FuncRetCode::HARDWARE_ERROR;
    }
    /// Step #2: Write ADC_CONFIG register, enabling AVG=1
    i2c_buffer[0] = 0x01;
    i2c_buffer[1] = 0xFB;
    i2c_buffer[2] = 0x69;
    i2c->WriteBytes(device_addr, i2c_buffer, 3);
    HAL::DelayMs(100);
    /// Step #3: Write Calibration register
    i2c_buffer[0] = 0x02;
    i2c_buffer[1] = (uint8_t) (current_cal_reg >> 8);
    i2c_buffer[2] = (uint8_t) current_cal_reg;
    i2c->WriteBytes(device_addr, i2c_buffer, 3);
    return FuncRetCode::OK;
}

void BusSenseINA237::Update()
{
    i2c_buffer[0] = 0x05;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    uint16_t temp = ((uint16_t) i2c_buffer[0] << 8) | i2c_buffer[1];
    voltage = (float)temp * 0.003125f;
    i2c_buffer[0] = 0x07;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    temp = ((uint16_t) i2c_buffer[0] << 8) | i2c_buffer[1];
    current = (float)((int16_t)((-(~temp + 1)) & 0xFFFF)) * current_lsb;
    if(reversed) current *= -1.0f;
}
}