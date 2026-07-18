#pragma once

#include "encoder_base.hpp"
#include "../Common/Interface/spi_base.hpp"
#include "../Common/Interface/i2c_base.hpp"
#include "../DataType/lookup_table.hpp"

namespace iFOC::Encoder
{
template<typename T>
class EncoderMT6701;

// I2C Freq: 1 MHz MAX
template<HAL::I2CImpl T>
class EncoderMT6701<T> final : public EncoderBase
{
public:
    EncoderMT6701(T* _i2c, real_t update_rate_hz) :
    EncoderBase("MT6701I2C", Type::ABSOLUTE_ENCODER, 1), i2c(_i2c)
    {
        update_rate_hz = _constrain(update_rate_hz, 0.5f, 1000.0f);
        if(MID_LOOP_TS > 0.0f)
        {
            // calculate update_tick
            update_tick = (uint16_t)((1.0f / MID_LOOP_TS) / update_rate_hz);
        }
    }
    explicit EncoderMT6701(T* _i2c) : EncoderMT6701(_i2c, 200.0f) {}
    ~EncoderMT6701() override = default;
    FuncRetCode Init(const uint8_t motor_id) override
    {
        result_valid = false;
        auto ret = i2c->Init();
        if(ret != FuncRetCode::OK) return ret;
        /// Step #1: Determine I2C ID, start from 0x06
        device_addr = 0x06;
        uint8_t temp = 0x00;
        ret = ReadReg(0x03, &temp);
        if(ret != FuncRetCode::OK)
        {
            // retry to 0x46
            device_addr = 0x46;
            ret = ReadReg(0x03, &temp);
            if(ret != FuncRetCode::OK) return ret;
        }
        /// Step #2: Try to read angle
        ret = ReadAbsAngleRad();
        if(ret != FuncRetCode::OK) return ret;
        last_compensated_angle_rad = compensated_single_round_angle_rad;
        multi_round_angle_rad = compensated_single_round_angle_rad;
        return FuncRetCode::OK;
    }
    void UpdateMid(const float Ts) override
    {
        if(update_tick > 0)
        {
            if(++update_tick_timer >= update_tick)
            {
                ReadAbsAngleRad();
                real_t delta = compensated_single_round_angle_rad - last_compensated_angle_rad;
                last_compensated_angle_rad = compensated_single_round_angle_rad;
                if(delta > PI)
                {
                    full_rotations--;
                    delta -= PI2;
                }
                else if(delta < -PI)
                {
                    full_rotations++;
                    delta += PI2;
                }
                multi_round_angle_rad = full_rotations * PI2 + compensated_single_round_angle_rad;
                const real_t vel = delta / Ts;
                angular_speed_rad_s = speed_lpf.GetOutput(vel, Ts);
                update_tick_timer = 0;
            }
        }
    }
private:
    FuncRetCode ReadReg(const uint8_t reg, uint8_t *data) const
    {
        *data = 0x00;
        uint8_t i2c_buffer = reg;
        auto ret = i2c->WriteBytes(device_addr, &i2c_buffer, 1);
        if(ret == FuncRetCode::OK)
        {
            i2c_buffer = 0;
            ret = i2c->ReadBytes(device_addr, &i2c_buffer, 1);
            if(ret == FuncRetCode::OK)
                *data = i2c_buffer;
        }
        return ret;
    }
    FuncRetCode ReadAbsAngleRad()
    {
        uint8_t reg_0x03 = 0;
        auto ret = ReadReg(0x03, &reg_0x03);
        if(ret == FuncRetCode::OK)
        {
            uint8_t reg_0x04 = 0;
            ret = ReadReg(0x04, &reg_0x04);
            if(ret == FuncRetCode::OK)
            {
                uint16_t now_angle_cnt = ((uint16_t)reg_0x03 << 6) | (((uint16_t)reg_0x04) & 0xFC) >> 2;
                if(sign_and_deduction_ratio < 0.0f) now_angle_cnt = (CPR - now_angle_cnt) & (CPR - 1);
                raw_single_round_angle_rad = (float)now_angle_cnt * PI2divCPR_f;
                // TODO: LUT here...
                compensated_single_round_angle_rad = raw_single_round_angle_rad;
                result_valid = true;
                return FuncRetCode::OK;
            }
        }
        return ret;
    }
    static constexpr uint16_t CPR = 16384;
    static constexpr real_t CPR_f = (real_t)CPR;
    static constexpr real_t PI2divCPR_f = PI2 / CPR_f;
    T* i2c = nullptr;
    real_t last_compensated_angle_rad = 0.0f;
    uint16_t update_tick = 0;
    uint16_t update_tick_timer = 0;
    // The default slave ID of MT6701 is b’0000110 in 7-bit binary form
    // (It could be programmed to b’1000110)
    uint8_t device_addr = 0x06; // 0x46 waiting to be tested
};

template<HAL::SPIImpl T>
class EncoderMT6701<T> final : public EncoderBase
{
public:
    explicit EncoderMT6701(T* _spi) :
    EncoderBase("MT6701SPI", Type::ABSOLUTE_ENCODER, 1), spi(_spi) {}
    ~EncoderMT6701() override = default;
    FuncRetCode Init(const uint8_t motor_id) override
    {
        result_valid = false;
        spi->SetCPOLCPHA(0, 1);
        spi->SetDataWidth(HAL::SPIBase::DataWidth::BYTE);
        spi->SetClock(15000000); // 15 MHz MAX
        if(const auto r = spi->Init(); r != FuncRetCode::OK) return FuncRetCode::HARDWARE_ERROR;

        // try to read nonlinear compensation lut
        char key[sizeof(NONLINEAR_LUT_DB_KEY_PREFIX) + 1];
        memcpy(key, NONLINEAR_LUT_DB_KEY_PREFIX, sizeof(NONLINEAR_LUT_DB_KEY_PREFIX) - 1);
        key[sizeof(NONLINEAR_LUT_DB_KEY_PREFIX) - 1] = motor_id + '0';
        key[sizeof(NONLINEAR_LUT_DB_KEY_PREFIX)] = '\0';

        auto buffer_size = BlobNVMStorage().GetKVSize(key);
        if(DataType::LookupTable::getTableSizeBySerializedSize(buffer_size) == NONLINEAR_LUT_POINTS)
        {
            uint8_t* deserialize_buffer = (uint8_t*)pvPortMalloc(buffer_size * sizeof(uint8_t));
            if(deserialize_buffer)
            {
                if(BlobNVMStorage().ReadNVM(key, deserialize_buffer, &buffer_size) == FuncRetCode::OK)
                {
                    nonlinear_lut.deserialize(deserialize_buffer, buffer_size);
                }
                vPortFree(deserialize_buffer);
                deserialize_buffer = nullptr;
            }
        }
        else // incorrect size, delete KV
        {
            BlobNVMStorage().ClearNVM(key);
        }

        return ReadAbsAngleRad();
    }
    void UpdateRT(const float Ts) override
    {
        ReadAbsAngleRad();
    }
    void UpdateMid(float Ts) override
    {
        real_t delta = compensated_single_round_angle_rad - last_compensated_angle_rad;
        last_compensated_angle_rad = compensated_single_round_angle_rad;
        if(delta > PI)
        {
            full_rotations--;
            delta -= PI2;
        }
        else if(delta < -PI)
        {
            full_rotations++;
            delta += PI2;
        }
        multi_round_angle_rad = full_rotations * PI2 + compensated_single_round_angle_rad;
        const real_t vel = delta / Ts;
        angular_speed_rad_s = speed_lpf.GetOutput(vel, Ts);
    }
private:
    FuncRetCode ReadAbsAngleRad()
    {
        // CRC Data Range: D[13:0] and Mg[3:0] total 18-bit, D[13] is the MSB, Mg[0] is the LSB
        // CRC polynomial: X6+X+1, MSB steam in first. (CRC6)
        uint8_t buf[3]{};
        if(const auto r = spi->ReadBytes(buf, 3); r != FuncRetCode::OK) return r;

        const uint8_t crc = crc6_table_0[buf[0] >> 2] ^
                            crc6_table_1[((buf[0] & 0x03) << 4) | (buf[1] >> 4)] ^
                            crc6_table_2[((buf[1] & 0x0F) << 2) | (buf[2] >> 6)];
        if(crc != (buf[2] & 0x3F)) return FuncRetCode::CRC_MISMATCH;

        uint16_t now_angle_cnt = ((uint16_t)buf[0] << 6) | ((uint16_t)buf[1] >> 2);
        if(sign_and_deduction_ratio < 0.0f) now_angle_cnt = (CPR - now_angle_cnt) & (CPR - 1);
        raw_single_round_angle_rad = (float)now_angle_cnt * PI2divCPR_f;
        if(nonlinear_lut.getTableSize() == NONLINEAR_LUT_POINTS)
        {
            const float nl_err = nonlinear_lut.lookupPeriodic(raw_single_round_angle_rad);
            compensated_single_round_angle_rad = normalize_rad(raw_single_round_angle_rad - nl_err);
        }
        else compensated_single_round_angle_rad = raw_single_round_angle_rad;
        result_valid = true;
        return FuncRetCode::OK;
    }
    static constexpr uint16_t CPR = 16384;
    static constexpr real_t CPR_f = (real_t)CPR;
    static constexpr real_t PI2divCPR_f = PI2 / CPR_f;
    static constexpr uint8_t crc6_table_0[64] = {
        0x00, 0x0F, 0x1E, 0x11, 0x3C, 0x33, 0x22, 0x2D, 0x3B, 0x34, 0x25, 0x2A, 0x07, 0x08, 0x19, 0x16,
        0x35, 0x3A, 0x2B, 0x24, 0x09, 0x06, 0x17, 0x18, 0x0E, 0x01, 0x10, 0x1F, 0x32, 0x3D, 0x2C, 0x23,
        0x29, 0x26, 0x37, 0x38, 0x15, 0x1A, 0x0B, 0x04, 0x12, 0x1D, 0x0C, 0x03, 0x2E, 0x21, 0x30, 0x3F,
        0x1C, 0x13, 0x02, 0x0D, 0x20, 0x2F, 0x3E, 0x31, 0x27, 0x28, 0x39, 0x36, 0x1B, 0x14, 0x05, 0x0A,
    };
    static constexpr uint8_t crc6_table_1[64] = {
        0x00, 0x05, 0x0A, 0x0F, 0x14, 0x11, 0x1E, 0x1B, 0x28, 0x2D, 0x22, 0x27, 0x3C, 0x39, 0x36, 0x33,
        0x13, 0x16, 0x19, 0x1C, 0x07, 0x02, 0x0D, 0x08, 0x3B, 0x3E, 0x31, 0x34, 0x2F, 0x2A, 0x25, 0x20,
        0x26, 0x23, 0x2C, 0x29, 0x32, 0x37, 0x38, 0x3D, 0x0E, 0x0B, 0x04, 0x01, 0x1A, 0x1F, 0x10, 0x15,
        0x35, 0x30, 0x3F, 0x3A, 0x21, 0x24, 0x2B, 0x2E, 0x1D, 0x18, 0x17, 0x12, 0x09, 0x0C, 0x03, 0x06,
    };
    static constexpr uint8_t crc6_table_2[64] = {
        0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
        0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
        0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
        0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02,
    };
    DataType::LookupTable nonlinear_lut;
    T* spi = nullptr;
    real_t last_compensated_angle_rad = 0.0f;
};
}
