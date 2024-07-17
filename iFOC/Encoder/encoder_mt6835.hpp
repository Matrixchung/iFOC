#ifndef _FOC_ENCODER_MT6835_H
#define _FOC_ENCODER_MT6835_H

#include "encoder_base.hpp"
#include "sliding_filter.h"
#include "lowpass_filter.h"
#include "spi_base.h"

// enum MT6835_CMD
// {
//     READ_REG        = 0x03, // 0011
//     WRITE_REG       = 0x06, // 0110
//     BURN_EEPROM     = 0x0C, // 1100
//     SET_ZEROPOINT   = 0x05, // 0101
//     CONTINUOUS_READ = 0x0A, // 1010
// };
// enum MT6835_REG
// {
//     USER_ID        = 0x001,
//     ANGLE_H        = 0x003,
//     ANGLE_M        = 0x004,
//     ANGLE_L_STATUS = 0x005,
//     CRC            = 0x006,
//     ABZ_RES_H      = 0x007,
//     ABZ_RES_L      = 0x008,
//     ZERO_POS_H     = 0x009,
//     ZERO_POS_L     = 0x00A,
//     UVW_SET        = 0x00B,
//     PWM_SET        = 0x00C,
//     ROT_DIR_HYST   = 0x00D,
//     IO_AUTOCAL     = 0x00E,
//     BW             = 0x011,
// };

// MT6835 SPI: CPOL = 1, CPHA = 1(2 Edge), MAX CLK = 16 MHz
// Write: 4 bit COMMAND + 12 bit REGISTER + 8 bit DATA
// Read:  --------------------------------+ 8 bit RETR

template<class T = SPIBase>
class EncoderMT6835 : public EncoderBase
{
public:
    EncoderMT6835(T& _spi) : spi(_spi)
    {
        static_assert(std::is_base_of<SPIBase, T>::value, "SPI Implementation must be derived from SPIBase");
    };
    bool Init() override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
    bool IsError() override {return overspeed_flag | low_magnetic_flag | undervoltage_flag;};
    uint8_t overspeed_flag = 0;
    uint8_t low_magnetic_flag = 0;
    uint8_t undervoltage_flag = 0;
    uint32_t raw_21bit_angle = 0;
private:
    T& spi;
    uint8_t crc_mismatch_count = 0;
    uint8_t last_angle_valid = 0;
    float single_round_angle_prev = -1.0f;
    uint8_t SPIWrite24Read8(uint8_t cmd, uint16_t reg, uint8_t data);
    uint8_t ReadAngle();
    uint8_t ReadAngleContinuous();
    uint8_t ReadAngleContinuousStart();
    uint8_t ReadAngleContinuousEnd();

    SlidingFilter angle_filter = SlidingFilter(10);
    // LowpassFilter angle_filter = LowpassFilter(0.001f);

    // virtual bool PortSPIInit() = 0;
    // virtual void PortSetCS(uint8_t state) = 0; // CS is low valid
    // virtual uint16_t PortSPIRead16(uint16_t reg, uint16_t *ret) = 0;
    // virtual uint16_t PortSPIRead8(uint8_t reg, uint8_t *ret) = 0;
};

template<class T>
bool EncoderMT6835<T>::Init()
{
    // direction = _dir;
    // max_velocity = max_vel;
    spi.ResetCS();
    // Turn off ABZ output
    // SPIWrite24Read8(0x06, 0x008, 0x02);
    // Turn off UVW output
    // SPIWrite24Read8(0x06, 0x00B, 0x10);
    // Set System Bandwidth = BaseBW x 2
    // SPIWrite24Read8(0x06, 0x011, 0x01);
    // Set Hysteresis Window
    // SPIWrite24Read8(0x06, 0x00D, 0x3);
    // ReadAngleContinuousStart();
    return true;
}

template<class T>
uint8_t EncoderMT6835<T>::SPIWrite24Read8(uint8_t cmd, uint16_t reg, uint8_t data)
{
    // PortSetCS(0);
    uint16_t segment = ((uint16_t)cmd << 12) | (reg & 0xFFF);
    uint8_t ret = 0;
    uint8_t txbuf[2] = {(uint8_t)(segment >> 8), (uint8_t)(segment & 0x00FF)};
    spi.WriteBytes(txbuf[0], 1);
    // PortSetCS(1);
    // PortSetCS(0);
    spi.WriteBytes(txbuf[1], 1);
    // PortSetCS(1);
    // PortSetCS(0);
    spi.ReadWriteBytes(&ret, &ret, 1);
    // PortSPIRead16(segment, &ret);
    // PortSPIRead16((uint16_t)data, &ret);
    // PortSetCS(1);
    return (uint8_t)ret;
}

template<class T>
uint8_t EncoderMT6835<T>::ReadAngle()
{
    // PortSetCS(0);
    // uint8_t result = 0;
    // uint16_t ret = 0;
    // uint8_t temp[3] = {0};
    // result |= (uint8_t)PortSPIRead16(0xA003, &ret);
    // result |= (uint8_t)PortSPIRead16(0x0000, &ret);
    // temp[0] = (uint8_t)(ret >> 8);
    // temp[1] = (uint8_t)ret;
    // raw_21bit_angle = ((uint32_t)ret << 5);
    // result |= (uint8_t)PortSPIRead16(0x0000, &ret);
    // temp[2] = (uint8_t)(ret >> 8);
    // PortSetCS(1);
    // if(result)
    // {
    //     raw_21bit_angle = 0;
    //     return 1;
    // }
    // if(getCRC8(temp, 3) == (uint8_t)ret)
    // {
    //     raw_21bit_angle |= (uint32_t)(ret & 0xF800);
    //     overspeed_flag = (uint8_t)(ret & 0x100);
    //     low_magnetic_flag = (uint8_t)(ret & 0x200);
    //     undervoltage_flag = (uint8_t)(ret & 0x400);
    //     return 0;
    // }
    // // crc_mismatch_count++;
    // raw_21bit_angle = 0;
    // return 1;
    spi.SetCS();
    uint8_t angle[3] = {0x00};
    // PortSPIRead8(0x003, &angle[0]);
    // PortSPIRead8(0x004, &angle[1]);
    // PortSPIRead8(0x005, &angle[2]);
    angle[0] = SPIWrite24Read8(0x03, 0x003, 0);
    angle[1] = SPIWrite24Read8(0x03, 0x004, 0);
    angle[2] = SPIWrite24Read8(0x03, 0x005, 0);
    raw_21bit_angle = angle[2] << 12 | angle[1] << 4 | angle[0] >> 4;
    spi.ResetCS();
    // if(result)
    // {
    //     raw_21bit_angle = 0;
    //     return 1;
    // }
    // if(getCRC8(temp, 3) == (uint8_t)ret)
    // {
    //     raw_21bit_angle |= (uint32_t)(ret & 0xF800);
    //     overspeed_flag = (uint8_t)(ret & 0x100);
    //     low_magnetic_flag = (uint8_t)(ret & 0x200);
    //     undervoltage_flag = (uint8_t)(ret & 0x400);
    //     return 0;
    // }
    // crc_mismatch_count++;
    // raw_21bit_angle = 0;
    // return 1;
    return 0;
}

template<class T>
uint8_t EncoderMT6835<T>::ReadAngleContinuous()
{
    uint16_t ret = 0;
    uint8_t temp[2] = {0};
    uint8_t temp_crc[3] = {0};
    // PortSPIRead16(0x0000, &ret);
    spi.ReadBytes(temp, 2);
    ret = (uint16_t)(temp[0] << 8) | (uint16_t)(temp[1]);
    temp_crc[0] = (uint8_t)(ret >> 8);
    temp_crc[1] = (uint8_t)ret;
    raw_21bit_angle = (uint32_t)(ret << 5);
    // PortSPIRead16(0x0000, &ret);
    spi.ReadBytes(temp, 2);
    ret = (uint16_t)(temp[0] << 8) | (uint16_t)(temp[1]);
    temp_crc[2] = (uint8_t)(ret >> 8);
    if(getCRC8(temp_crc, 3) == (uint8_t)ret)
    {
        raw_21bit_angle |= (uint32_t)(ret & 0xF800);
        overspeed_flag = (uint8_t)(ret & 0x100);
        low_magnetic_flag = (uint8_t)(ret & 0x200);
        undervoltage_flag = (uint8_t)(ret & 0x400);
        return 0;
    }
    raw_21bit_angle = 0;
    crc_mismatch_count++;
    return 1;
}

template<class T>
uint8_t EncoderMT6835<T>::ReadAngleContinuousStart()
{
    spi.SetCS();
    // uint16_t ret = 0;
    // return (uint8_t)PortSPIRead16(0xA003, &ret);
    uint8_t temp = {0xA0, 0x03};
    return spi.WriteBytes(temp, 2);
}

template<class T>
uint8_t EncoderMT6835<T>::ReadAngleContinuousEnd()
{
    spi.ResetCS();
    return 0;
}

template<class T>
void EncoderMT6835<T>::Update(float Ts)
{
    // if(!ReadAngleContinuous()) // read angle continuously
    if(!ReadAngle())
    {
        // single_round_angle = normalize_rad((float)direction * ((float)raw_21bit_angle / 2097152.0f * PI2));
        single_round_angle = normalize_rad(((float)raw_21bit_angle / 2097152.0f * PI2));
        // single_round_angle = normalize_rad(angle_filter.GetOutput(((float)direction * ((float)raw_21bit_angle / 2097152.0f * PI2))));
        if(last_angle_valid && single_round_angle_prev >= 0.0f)
        {
            float delta = single_round_angle - single_round_angle_prev;
            if(delta > 0.8f * PI2) full_rotations -= 1;
            else if(delta < -0.8f * PI2) full_rotations += 1;
            raw_angle = full_rotations * PI2 + single_round_angle;
        }
        single_round_angle_prev = single_round_angle;
        last_angle_valid = 1;
    }
    else
    {
        last_angle_valid = 0;
        if(crc_mismatch_count >= 30)
        {
            // ReadAngleContinuousEnd();
            // ReadAngleContinuousStart();
        }
    }
}

template<class T>
void EncoderMT6835<T>::UpdateMidInterval(float Ts)
{
    // velocity = speed_pll.Calculate(single_round_angle, Ts);
}

template<class T>
bool EncoderMT6835<T>::IsCalibrated()
{
    return true;
}

#endif