#pragma once

#include "bus_sense_base.hpp"
#include "../Common/Interface/i2c_base.hpp"
#include "../DataType/board_config.hpp"
#include "../Port/hal_clock.hpp"

namespace iFOC::Sense
{
#define config iFOC::BoardConfig.GetConfig()
class BusSenseINA226 final : public BusSenseBase
{
private:
    using I2CBase = iFOC::HAL::I2CBase;
public:
    BusSenseINA226() = delete;
    BusSenseINA226(I2CBase* _i2c,
                   uint8_t _addr,
                   float _max,
                   float _shunt) :
           i2c(_i2c), device_addr(_addr), max_current(_max), rshunt_ohm(_shunt), current_lsb(_max / 32768.0f)
    {
        // power_lsb = current_lsb * 25.0f;
        current_cal_reg = (uint16_t)(0.00512f / (current_lsb * rshunt_ohm));
        if(!BETWEEN(max_current, 0.0f, 1000.0f) ||
           !BETWEEN(rshunt_ohm, 0.00001f, 1.0f))
        {
            misconfigured_area |= to_underlying(MisconfiguredArea::BUS_SENSE_CONFIGS_OUT_OF_RANGE);
        }
    };
    explicit BusSenseINA226(I2CBase* _i2c):
             BusSenseINA226(_i2c,
                            0x40,
                            config.bus_max_positive_current() * 2.0f,
                            config.bus_sense_shunt_ohm()) {};
    FuncRetCode Init() final;
    void Update() final;
private:
    I2CBase* i2c = nullptr;
    uint8_t device_addr = 0x40;
    float max_current = 10.0f;
    float rshunt_ohm = 0.002f;
    uint8_t i2c_buffer[3] = {0};
    uint16_t current_cal_reg = 0;
    float current_lsb = 0.001f;
    // float power_lsb = 0.025f;
};

FuncRetCode BusSenseINA226::Init()
{
    i2c->Init();
    /// Step #1: Read MANUFACTURER_ID register, it should be 0x5449
    i2c_buffer[0] = 0xFE;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    if(i2c_buffer[0] != 0x54 || i2c_buffer[1] != 0x49) return FuncRetCode::HARDWARE_ERROR;
    /// Step #1: Write Config register
    i2c_buffer[0] = 0x00;
    i2c_buffer[1] = 0x45;
    i2c_buffer[2] = 0x27;
    i2c->WriteBytes(device_addr, i2c_buffer, 3);
    iFOC::HAL::DelayMs(100);
    /// Step #2: Write Calibration register
    i2c_buffer[0] = 0x05;
    i2c_buffer[1] = (uint8_t)(current_cal_reg >> 8);
    i2c_buffer[2] = (uint8_t)current_cal_reg;
    i2c->WriteBytes(device_addr, i2c_buffer, 3);
    return FuncRetCode::OK;
}

void BusSenseINA226::Update()
{
    i2c_buffer[0] = 0x02;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    uint16_t temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    voltage = (float)temp * 0.00125f;
    i2c_buffer[0] = 0x04;
    i2c->WriteBytes(device_addr, i2c_buffer, 1);
    i2c->ReadBytes(device_addr, i2c_buffer, 2);
    temp = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    current = (float)temp * current_lsb;
    if(config.bus_sense_dir_reversed()) current *= -1.0f;
}

#undef config
}