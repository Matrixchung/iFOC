#ifndef _FOC_ENCODER_INTERPOLATION_H
#define _FOC_ENCODER_INTERPOLATION_H

#include "encoder_base.hpp"

// Interpolation wrapper

class EncoderInterp : public EncoderBase
{
public:
    EncoderInterp(EncoderBase& base): wrapped(&base) {};
    EncoderInterp(EncoderBase& base, uint8_t _downsample) : wrapped(&base), downsample(_downsample) {};
    EncoderBase *wrapped;
    bool Init() final;
    void Update(float Ts) final;
    void UpdateMidInterval(float Ts) final;
    bool IsCalibrated() final { return wrapped->IsCalibrated(); };
private:
    uint8_t downsample = 5;
    uint8_t downsample_cnt = 0;
};

bool EncoderInterp::Init()
{
    return wrapped->Init();
}

void EncoderInterp::Update(float Ts)
{
    if(downsample_cnt++ >= downsample)
    {
        downsample_cnt = 0;
        wrapped->Update(Ts);
    }
    velocity = wrapped->velocity;
    float interp = _constrain(velocity * Ts, -_PI_3, _PI_3);
    single_round_angle = wrapped->single_round_angle + interp;
    full_rotations = wrapped->full_rotations;
    if(single_round_angle < 0.0f)
    {
        single_round_angle += PI2;
        full_rotations -= 1;
    }
    else if(single_round_angle >= PI2)
    {
        single_round_angle -= PI2;
        full_rotations += 1;
    }
    raw_angle = full_rotations * PI2 + single_round_angle;
}

void EncoderInterp::UpdateMidInterval(float Ts)
{
    wrapped->UpdateMidInterval(Ts);
}

#endif