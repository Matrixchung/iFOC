#ifndef _FOC_ENCODER_ABZ_BASE_H
#define _FOC_ENCODER_ABZ_BASE_H

#include "encoder_base.hpp"

class EncoderABZBase : public EncoderBase
{
public:
    EncoderABZBase(uint32_t _cpr): cpr(_cpr) {};
    bool Init() override;
    void Update(float Ts) override;
    void UpdateMidInterval(float Ts) override;
    bool IsCalibrated() override;
protected:
    int8_t direction = FOC_DIR_POS;
// #if ENCODER_ABZ_USE_PLL
//     SpeedPLL speed_pll;
// #else
    // SlidingFilter sliding_filter = SlidingFilter(20);
    // LowpassFilter speed_filter = LowpassFilter(0.0001f);
    // int last_rotations = 0;
    float last_angle = 0.0f;
    // float last_velocity = 0.0f;
// #endif
    uint32_t cpr = 4095;
    short pulse = 0;
    bool zero_signal = false;
    virtual bool PortInit() = 0;
    virtual void PortUpdateDirPulse() = 0;
};

bool EncoderABZBase::Init()
{
// #if ENCODER_ABZ_USE_PLL
//     speed_pll.pi_controller.Kp = 350.0f;
//     speed_pll.pi_controller.Ki = 90000.0f;
//     speed_pll.pi_controller.limit = max_velocity;
//     speed_pll.lpf.Tf = 0.001f;
// #endif
    return PortInit();
}

void EncoderABZBase::Update(float Ts)
{
    // PortUpdateDirPulse(): update direction, pulse
    PortUpdateDirPulse();
    single_round_angle = normalize_rad(((float)pulse / (float)cpr) * PI2);
    if(zero_signal)
    {
        float delta = single_round_angle - last_angle;
        if(delta > 0.6f * PI2) full_rotations--;
        else if(delta < -0.6f * PI2) full_rotations++;
    }
    last_angle = single_round_angle;
    raw_angle = full_rotations * PI2 + single_round_angle;
}

void EncoderABZBase::UpdateMidInterval(float Ts)
{
// #if ENCODER_ABZ_USE_PLL
//     // velocity = speed_pll.Calculate(single_round_angle, Ts);
//     ;
// #else
    // float vel = ((float)(full_rotations - last_rotations) * PI2 + (single_round_angle - last_angle)) / Ts;
    // last_angle = single_round_angle;
    // last_rotations = full_rotations;
    // vel = sliding_filter.GetOutput(vel);
    // if(ABS(vel) >= 1.0f && ABS(vel - last_velocity) >= 100.0f) vel = last_velocity;
    // last_velocity = vel;
    // return speed_filter.GetOutput(vel, Ts);
// #endif
}

bool EncoderABZBase::IsCalibrated()
{
    return zero_signal;
}

#endif