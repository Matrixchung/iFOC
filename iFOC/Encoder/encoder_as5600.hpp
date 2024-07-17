#ifndef _FOC_ENCODER_AS5600_BASE_H
#define _FOC_ENCODER_AS5600_BASE_H

#include "encoder_base.hpp"
#include "i2c_base.h"

template<class T = I2CBase>
class EncoderAS5600 : public EncoderBase
{
public:
    EncoderAS5600(T& _i2c, uint8_t _addr) : i2c(_i2c), device_addr(_addr)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
    };
    EncoderAS5600(T& _i2c) : i2c(_i2c)
    {
        static_assert(std::is_base_of<I2CBase, T>::value, "I2C Implementation must be derived from I2CBase");
    };
    bool Init() override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
private:
    T& i2c;
    uint8_t device_addr = 0x36;
    uint8_t i2c_buffer[2] = {0};
    uint16_t raw_12bit_angle = 0;
    float single_round_angle_prev = -1.0f;
    // SlidingFilter sliding_filter = SlidingFilter(10);
    LowpassFilter speed_filter = LowpassFilter(0.0008f);
    float last_raw_angle = 0.0f;
    // uint8_t repeat_counter = 0; // runs at 2KHz
};

template<class T>
bool EncoderAS5600<T>::Init()
{
    i2c.Init();
    return true;
}

template<class T>
void EncoderAS5600<T>::Update(float Ts)
{
    ; // update interval may be too short for I2C communication
}

template<class T>
void EncoderAS5600<T>::UpdateMidInterval(float Ts)
{
    uint8_t temp = 0x0C;
    i2c.WriteBytes(device_addr, &temp, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    raw_12bit_angle = ((uint16_t)i2c_buffer[0] << 8) | i2c_buffer[1];
    single_round_angle = normalize_rad((float)raw_12bit_angle / 4095.0f * PI2);
    if(single_round_angle_prev >= 0.0f)
    {
        float delta = single_round_angle - single_round_angle_prev;
        if(delta > 0.8f * PI2) full_rotations -= 1;
        else if(delta < -0.8f * PI2) full_rotations += 1;
        raw_angle = full_rotations * PI2 + single_round_angle;
        float vel = (float)(raw_angle - last_raw_angle) / Ts;
        // vel = sliding_filter.GetOutput(vel);
        velocity = speed_filter.GetOutput(vel, Ts);
        last_raw_angle = raw_angle;
        // velocity = sliding_filter.GetOutput(vel);
    }
    single_round_angle_prev = single_round_angle;
    // repeat_counter = 0;
}

template<class T>
bool EncoderAS5600<T>::IsCalibrated()
{
    return true;
}

#endif