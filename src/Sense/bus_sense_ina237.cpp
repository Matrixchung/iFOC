#include "bus_sense_ina237.hpp"

#define config iFOC::BoardConfig().GetConfig()

namespace iFOC::Sense
{
BusSenseINA237::BusSenseINA237(I2CBase *_i2c, uint8_t _addr, float _max, float _shunt, bool rev) :
        i2c(_i2c), device_addr(_addr), max_current(_max), rshunt_ohm(_shunt), reversed(rev)
{
    current_lsb = max_current / 32768.0f;
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

    auto ret = i2c->Init();
    if(ret != FuncRetCode::OK) return ret;

    /// Step #1: Read MANUFACTURER_ID register, it should be 0x5449
    uint16_t temp = 0x00;
    ret = ReadReg(0x3E, &temp);
    if(ret != FuncRetCode::OK) return ret;
    if(temp != 0x5449)
    {
        if(temp > 0x00) return FuncRetCode::CRC_MISMATCH;
        return FuncRetCode::HARDWARE_ERROR;
    }

    /// Step #2: Write ADC_CONFIG register, enabling AVG=1
    adc_config adc_cfg{};
    adc_cfg.bit.avg = 1; // average times = 4
    adc_cfg.bit.vtct = 6; // temperature conv time = 2074us
    adc_cfg.bit.vshct = 5; // Vshunt conv time = 1052us
    adc_cfg.bit.vbusct = 6; // Vbus conv time = 2074us
    adc_cfg.bit.mode = 0xF; // mode: continuous Vbus, Vshunt & Temp
    ret = WriteReg(0x01, adc_cfg.reg);
    if(ret != FuncRetCode::OK) return ret;
    HAL::DelayMs(10);

    /// Step #3: Write Calibration register
    /// SHUNT_CAL = 819.2 * 10^6 * current_lsb * rshunt
    /// CURRENT_LSB = max_current / 2^15
    shunt_cal s_cal{};
    s_cal.bit.shunt_cal = (uint16_t)(819200000.0f * current_lsb * rshunt_ohm);
    ret = WriteReg(0x02, s_cal.reg);
    if(ret != FuncRetCode::OK) return ret;

    return FuncRetCode::OK;
}

FuncRetCode BusSenseINA237::Update()
{
    volatile uint16_t temp = 0;
    /// Step #1: Read VBUS
    auto ret = ReadReg(0x05, (uint16_t*)&temp);
    if(ret != FuncRetCode::OK)
    {
        voltage = 0.0f;
        current = 0.0f;
        return ret;
    }
    voltage = (float)temp * 0.003125f;

    /// Step #2: Read CURRENT
    ret = ReadReg(0x07, (uint16_t*)&temp);
    if(ret != FuncRetCode::OK)
    {
        voltage = 0.0f;
        current = 0.0f;
        return ret;
    }
    current = (float)((int16_t)((-(~temp + 1)) & 0xFFFF)) * current_lsb;
    if(reversed) current *= -1.0f;

    return FuncRetCode::OK;
}

FuncRetCode BusSenseINA237::WriteReg(uint8_t reg, uint16_t data) const
{
    uint8_t i2c_buffer[3]{reg, (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF)};
    return i2c->WriteBytes(device_addr, i2c_buffer, 3);
}

FuncRetCode BusSenseINA237::ReadReg(uint8_t reg, uint16_t* data) const
{
    *data = 0x00;
    uint8_t i2c_buffer[2]{};
    i2c_buffer[0] = reg;
    auto ret = i2c->WriteBytes(device_addr, i2c_buffer, 1); // set register pointer
    if(ret == FuncRetCode::OK)
    {
        i2c_buffer[0] = 0x00;
        ret = i2c->ReadBytes(device_addr, i2c_buffer, 2);
        if(ret == FuncRetCode::OK)
            *data = ((uint16_t) i2c_buffer[0] << 8) | i2c_buffer[1];
    }
    return ret;
}
}
