#ifndef _FOC_ENCODER_AS5048B_BASE_H
#define _FOC_ENCODER_AS5048B_BASE_H

#include "encoder_base.hpp"
#include "i2c_base.h"

// A1, A2, ADDR = 10000(A2)(A1)b
// 0x40, 0x41, 0x42, 0x43

template<I2CImpl T>
class EncoderAS5048B : public EncoderBase
{
public:
    EncoderAS5048B(T& _i2c, uint8_t _addr) : i2c(_i2c), device_addr(_addr) {};
    bool Init() final;
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final;
    bool IsCalibrated() final;
private:
    T& i2c;
    uint8_t device_addr = 0x00;
    uint8_t i2c_buffer[2] = {0};
    uint16_t raw_14bit_angle = 0;
    float single_round_angle_prev = -1.0f;
    SlidingFilter sliding_filter = SlidingFilter(10);
    LowpassFilter speed_filter = LowpassFilter(0.001f);
    float last_raw_angle = 0.0f;
    // uint8_t repeat_counter = 0; // runs at 2KHz
};

template<I2CImpl T>
bool EncoderAS5048B<T>::Init()
{
    i2c.Init();
    return true;
}

template<I2CImpl T>
void EncoderAS5048B<T>::Update(float Ts)
{
    ; // update interval may be too short for I2C communication
}

template<I2CImpl T>
void EncoderAS5048B<T>::UpdateMidInterval(float Ts)
{
    uint8_t temp = 0xFE;
    i2c.WriteBytes(device_addr, &temp, 1);
    i2c.ReadBytes(device_addr, i2c_buffer, 2);
    raw_14bit_angle = ((uint16_t)(i2c_buffer[1]) << 6); // the two bytes are f**king reversed
    raw_14bit_angle += i2c_buffer[0] & 0x3F;
    single_round_angle = normalize_rad((float)raw_14bit_angle / 16384.0f * PI2);
    if(single_round_angle_prev >= 0.0f)
    {
        float delta = single_round_angle - single_round_angle_prev;
        if(delta > 0.8f * PI2) full_rotations -= 1;
        else if(delta < -0.8f * PI2) full_rotations += 1;
        raw_angle = full_rotations * PI2 + single_round_angle;
        float vel = (float)(raw_angle - last_raw_angle) / Ts;
        // vel = sliding_filter.GetOutput(vel);
        vel = speed_filter.GetOutput(vel, Ts);
        last_raw_angle = raw_angle;
        velocity = sliding_filter.GetOutput(vel);
    }
    single_round_angle_prev = single_round_angle;
}

template<I2CImpl T>
bool EncoderAS5048B<T>::IsCalibrated()
{
    return true;
}

#endif