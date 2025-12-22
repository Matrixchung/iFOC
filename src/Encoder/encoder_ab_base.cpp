#include "encoder_ab_base.hpp"

namespace iFOC::Encoder
{
EncoderABBase::EncoderABBase(uint32_t _cpr) : EncoderBase("EncAB", Type::INCREMENTAL_ENCODER, 1), cpr(_cpr) {}

void EncoderABBase::UpdateRT(float Ts)
{
    UpdatePulse();
    single_round_angle_rad = normalize_rad(PI2 * (real_t)pulse / (real_t)cpr);
    if(single_round_angle_rad_prev >= 0.0f)
    {
        real_t delta = single_round_angle_rad - single_round_angle_rad_prev;
        if(delta > 0.8f * PI2) full_rotations--;
        else if(delta < -0.8f * PI2) full_rotations++;
        multi_round_angle_rad = (real_t)full_rotations * PI2 + single_round_angle_rad;
    }
    single_round_angle_rad_prev = single_round_angle_rad;
}

void EncoderABBase::UpdateMid(float Ts)
{
    if(single_round_angle_rad_mid_prev >= 0.0f)
    {
        real_t velocity = ((real_t)(full_rotations - last_rotations) * PI2 + (single_round_angle_rad - single_round_angle_rad_mid_prev)) / Ts;
        last_rotations = full_rotations;
        angular_speed_rad_s = speed_filter.GetOutput(velocity, Ts);
    }
    single_round_angle_rad_mid_prev = single_round_angle_rad;
}
}
