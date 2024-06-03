#ifndef _FOC_ENCODER_AB_BASE_H
#define _FOC_ENCODER_AB_BASE_H

#include "encoder_base.h"

#define ENCODER_AB_USE_PLL 0

class EncoderABBase : public EncoderBase
{
public:
    EncoderABBase(uint32_t _cpr): cpr(_cpr) {};
    bool Init(float max_vel) override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override { return single_round_angle_prev != -1.0f; };
    short pulse = 0;
protected:
    int8_t direction = FOC_DIR_POS;
#if ENCODER_AB_USE_PLL
    SpeedPLL speed_pll = SpeedPLL(0.0f, 0.0f, 0.0f);
#else
    SlidingFilter sliding_filter = SlidingFilter(20);
    LowpassFilter speed_filter = LowpassFilter(0.0001f);
    int last_rotations = 0;
    float last_angle = 0.0f;
    float last_velocity = 0.0f;
#endif
    uint32_t cpr = 4095;
    
    float single_round_angle_prev = -1.0f;
    virtual bool PortInit() = 0;
    virtual void PortUpdateDirPulse() = 0;
};

bool EncoderABBase::Init(float max_vel)
{
    max_velocity = max_vel;
#if ENCODER_AB_USE_PLL
    speed_pll = SpeedPLL(800.0f, 30000.0f, max_velocity);
#endif
    return PortInit();
}

void EncoderABBase::Update(float Ts)
{
    // PortUpdateDirPulse(): update direction, pulse
    PortUpdateDirPulse();
    single_round_angle = normalize_rad(((float)pulse / (float)cpr) * PI2);
    if(single_round_angle_prev >= 0.0f)
    {
        float delta = single_round_angle - single_round_angle_prev;
        if(delta > 0.8f * PI2) full_rotations -= 1;
        else if(delta < -0.8f * PI2) full_rotations += 1;
        raw_angle = full_rotations * PI2 + single_round_angle;
    }
    single_round_angle_prev = single_round_angle;
#if ENCODER_AB_USE_PLL
    velocity = speed_pll.Calculate(single_round_angle, Ts);
#endif
}

void EncoderABBase::UpdateMidInterval(float Ts)
{
#if ENCODER_AB_USE_PLL
    // velocity = speed_pll.Calculate(single_round_angle, Ts);
    ;
#else
    float vel = ((float)(full_rotations - last_rotations) * PI2 + (single_round_angle - last_angle)) / Ts;
    last_angle = single_round_angle;
    last_rotations = full_rotations;
    vel = sliding_filter.GetOutput(vel);
    // if(ABS(vel) >= 1.0f && ABS(vel - last_velocity) >= 200.0f) vel = last_velocity;
    last_velocity = vel;
    velocity = speed_filter.GetOutput(vel, Ts);
#endif
}

#endif