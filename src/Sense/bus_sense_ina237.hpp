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
    BusSenseINA237(I2CBase *_i2c, uint8_t _addr, float _max, float _shunt, bool rev);
    explicit BusSenseINA237(I2CBase *_i2c);
    BusSenseINA237(I2CBase *_i2c, uint8_t _addr, bool rev);
    FuncRetCode Init() override;
    FuncRetCode Update() override;
private:
    typedef union config // address: 0x00
    {
        uint16_t reg{};
        struct
        {
            uint16_t          : 4; /* [3:0] */
            uint16_t adcrange : 1; /* [4] */
            uint16_t          : 1; /* [5] */
            uint16_t convdly  : 8; /* [13:6] */
            uint16_t          : 1; /* [14] */
            uint16_t rst      : 1; /* [15] */
        } bit;
    } config;
    typedef union adc_config // address: 0x01
    {
        uint16_t reg{};
        struct
        {
            uint16_t avg    : 3; /* [2:0] */
            uint16_t vtct   : 3; /* [5:3] */
            uint16_t vshct  : 3; /* [8:6] */
            uint16_t vbusct : 3; /* [11:9] */
            uint16_t mode   : 4; /* [15:12] */
        } bit;
    } adc_config;
    typedef union shunt_cal // address: 0x02
    {
        uint16_t reg{};
        struct
        {
            uint16_t shunt_cal : 15; /* [14:0] */
            uint16_t           : 1;  /* [15] */
        } bit;
    } shunt_cal;
    FuncRetCode WriteReg(uint8_t reg, uint16_t data) const;
    FuncRetCode ReadReg(uint8_t reg, uint16_t* data) const;
    I2CBase *i2c = nullptr;
    uint8_t device_addr = 0x40;
    float max_current = 10.0f;
    float rshunt_ohm = 0.002f;
    float current_lsb = 0.001f;
    // float power_lsb = 0.025f;
    bool reversed = false;
};
}