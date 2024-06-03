#ifndef _FOC_ENCODER_ABZ_BASE_H
#define _FOC_ENCODER_ABZ_BASE_H

#include "encoder_base.h"

#define ENCODER_ABZ_USE_PLL 1

class EncoderABZBase : public EncoderBase
{
public:
    EncoderABZBase(uint32_t _cpr): cpr(_cpr) {};
    bool Init(float max_vel) override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
protected:
    int8_t direction = FOC_DIR_POS;
#if ENCODER_ABZ_USE_PLL
    SpeedPLL speed_pll = SpeedPLL(0.0f, 0.0f, 0.0f);
#else
    SlidingFilter sliding_filter = SlidingFilter(20);
    LowpassFilter speed_filter = LowpassFilter(0.0001f);
    int last_rotations = 0;
    float last_angle = 0.0f;
    float last_velocity = 0.0f;
#endif
    uint32_t cpr = 4095;
    short pulse = 0;
    bool zero_signal = false;
    virtual bool PortInit() = 0;
    virtual void PortUpdateDirPulse() = 0;
    virtual void PortZeroSignalIRQ() = 0;
};

bool EncoderABZBase::Init(float max_vel)
{
    max_velocity = max_vel;
#if ENCODER_ABZ_USE_PLL
    speed_pll = SpeedPLL(350.0f, 90000.0f, max_velocity);
#endif
    return PortInit();
}

void EncoderABZBase::Update(float Ts)
{
    // PortUpdateDirPulse(): update direction, pulse
    PortUpdateDirPulse();
    single_round_angle = normalize_rad(((float)pulse / (float)cpr) * PI2);
    raw_angle = full_rotations * PI2 + single_round_angle;
#if ENCODER_ABZ_USE_PLL
    velocity = speed_pll.Calculate(single_round_angle, Ts);
#endif
}

void EncoderABZBase::UpdateMidInterval(float Ts)
{
#if ENCODER_ABZ_USE_PLL
    // velocity = speed_pll.Calculate(single_round_angle, Ts);
    ;
#else
    float vel = ((float)(full_rotations - last_rotations) * PI2 + (single_round_angle - last_angle)) / Ts;
    last_angle = single_round_angle;
    last_rotations = full_rotations;
    vel = sliding_filter.GetOutput(vel);
    if(ABS(vel) >= 1.0f && ABS(vel - last_velocity) >= 100.0f) vel = last_velocity;
    last_velocity = vel;
    return speed_filter.GetOutput(vel, Ts);
#endif
}

bool EncoderABZBase::IsCalibrated()
{
    return zero_signal;
}

#endif