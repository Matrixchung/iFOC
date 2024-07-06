#ifndef _FOC_ENCODER_AS5048A_BASE_H
#define _FOC_ENCODER_AS5048A_BASE_H

#include "encoder_base.hpp"

class EncoderAS5048ABase : public EncoderBase
{
public:
    bool Init() override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
    uint16_t raw_14bit_angle = 0;
    uint8_t parity_error_flag = 0;
    uint8_t comm_error_flag = 0;
protected:
    uint8_t last_angle_valid = 0;
    float single_round_angle_prev = -1.0f;
    uint8_t ReadAngle();
    virtual bool PortSPIInit() = 0;
    virtual void PortSetCS(uint8_t state) = 0; // CS is low valid
    virtual uint16_t PortSPIRead16(uint16_t reg, uint16_t *ret) = 0;
};

bool EncoderAS5048ABase::Init()
{
    // direction = _dir;
    PortSetCS(1);
    if(!PortSPIInit()) return false;
    if(ReadAngle()) return false;
    return true;
}

uint8_t EncoderAS5048ABase::ReadAngle()
{   
    PortSetCS(0);
    uint16_t ret = 0;
    if(PortSPIRead16(0xFFFF, &ret)) ret |= 0x4000;
    PortSetCS(1);
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

void EncoderAS5048ABase::Update(float Ts)
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

void EncoderAS5048ABase::UpdateMidInterval(float Ts)
{
    velocity = 0.0f;
}

bool EncoderAS5048ABase::IsCalibrated()
{
    return true;
}

#endif