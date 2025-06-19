#pragma once

#include "bus_sense_base.hpp"
#include "../Common/Interface/i2c_base.hpp"
#include "../DataType/board_config.hpp"

namespace iFOC::Sense
{
class BusSenseINA237 final : public BusSenseBase
{
private:
    using I2CBase = iFOC::HAL::I2CBase;
public:
    BusSenseINA237() = delete;
    BusSenseINA237(I2CBase *_i2c, uint8_t _addr, float _max, float _shunt);
    explicit BusSenseINA237(I2CBase *_i2c);
    FuncRetCode Init() final;
    void Update() final;
private:
    I2CBase *i2c = nullptr;
    uint8_t device_addr = 0x40;
    float max_current = 10.0f;
    float rshunt_ohm = 0.002f;
    uint8_t i2c_buffer[3] = {0};
    uint16_t current_cal_reg = 0;
    float current_lsb = 0.001f;
    // float power_lsb = 0.025f;
};
}