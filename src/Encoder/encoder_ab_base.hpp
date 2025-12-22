#pragma once

#include "encoder_base.hpp"
#include "lowpass_filter.hpp"

namespace iFOC::Encoder
{
class EncoderABBase : public EncoderBase
{
public:
    explicit EncoderABBase(uint32_t _cpr);
    void UpdateRT(float Ts) final;
    void UpdateMid(float Ts) final;
    // EncoderAB needs to implement: FuncRetCode Init(), void UpdatePulse()
    short pulse = 0;
protected:
    uint32_t cpr = 0;
    real_t single_round_angle_rad_prev = -1.0f;
    real_t single_round_angle_rad_mid_prev = -1.0f; // used in speed calculation
    Filter::LowpassFilter speed_filter = Filter::LowpassFilter(100.0f); // 100Hz lowpass
    long long last_rotations = 0;
    virtual void UpdatePulse() = 0;
};
}