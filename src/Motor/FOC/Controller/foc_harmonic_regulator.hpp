#pragma once

#include "foc_types.hpp"

namespace iFOC::FOC
{
// Proportional-Resonant Regulator (Phase-current 5/7/11/13 harmonic)
// https://zhuanlan.zhihu.com/p/699288152
class HarmonicRegulator
{
public:
    HarmonicRegulator(float _kr, float _wc, float _limit);
    float GetOutput(float error, float w0, float phase_comp_rad, float Ts);
    float GetOutputWithSinCos(float error, float w0, float sin_w0_Ts, float cos_w0_Ts, float sin_comp, float cos_comp, float Ts);
    void Reset();
    float Kr = 5.0f;
    float wc = 5.0f;
    float output_limit = 0.0f;
    float max_step_current = 0.0f; // Disable regulation if abs(error) >= max_step_current
private:
    float error_1 = 0.0f;
    float error_2 = 0.0f;
    float y_1 = 0.0f;
    float y_2 = 0.0f;
};

}