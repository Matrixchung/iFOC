#ifndef _FOC_ENCODER_AS5048A_H
#define _FOC_ENCODER_AS5048A_H

#include "encoder_base.hpp"
#include "spi_base.h"

template<SPIImpl T>
class EncoderAS5048A : public EncoderBase
{
public:
    explicit EncoderAS5048A(T& _spi) : spi(_spi) {};
    bool Init() final;
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final;
    bool IsCalibrated() final;
    uint16_t raw_14bit_angle = 0;
    uint8_t parity_error_flag = 0;
    uint8_t comm_error_flag = 0;
private:
    T& spi;
    uint8_t last_angle_valid = 0;
    float single_round_angle_prev = -1.0f;
    uint8_t ReadAngle();
};

template<SPIImpl T>
bool EncoderAS5048A<T>::Init()
{
    spi.ResetCS();
    if(ReadAngle()) return false;
    return true;
}

template<SPIImpl T>
uint8_t EncoderAS5048A<T>::ReadAngle()
{   
    spi.SetCS();
    uint8_t data_u8[2] = {0xFF, 0xFF};
    uint16_t ret = 0;
    if(spi.ReadWriteBytes(data_u8, data_u8, 2))
    {
        ret = (uint16_t)data_u8[0] << 8 | (uint16_t)data_u8[1];
        ret |= 0x4000;
    }
    else
    {
        spi.ResetCS();
        return 1;
    }
    spi.ResetCS();
    comm_error_flag = (uint8_t)(ret & 0x4000);
    if(comm_error_flag) return 1;
    uint8_t parity = 0;
    for(uint8_t i = 0; i < 15; i++) if(ret & (0x01 << i)) parity = !parity;
    if(parity == (uint8_t)(ret & 0x8000))
    {
        raw_14bit_angle = ret & 0x3FFF;
        parity_error_flag = 0;
        return 0;
    }
    parity_error_flag = 1;
    return 1;
}

template<SPIImpl T>
void EncoderAS5048A<T>::Update(float Ts)
{
    if(!ReadAngle())
    {
        single_round_angle = normalize_rad(((float)raw_14bit_angle / 16384.0f * PI2));
        raw_angle = single_round_angle;
        if(last_angle_valid && single_round_angle_prev >= 0.0f)
        {
            float delta = single_round_angle - single_round_angle_prev;
            if(delta > 0.8f * PI2) full_rotations -= 1;
            else if(delta < -0.8f * PI2) full_rotations += 1;
            raw_angle += full_rotations * PI2;
        }
        single_round_angle_prev = single_round_angle;
        last_angle_valid = 1;
    }
    else
    {
        last_angle_valid = 0;
    }
}

template<SPIImpl T>
void EncoderAS5048A<T>::UpdateMidInterval(float Ts)
{
    // velocity = 0.0f;
}

template<SPIImpl T>
bool EncoderAS5048A<T>::IsCalibrated()
{
    return true;
}

#endif