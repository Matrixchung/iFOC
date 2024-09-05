#ifndef _FOC_SPEED_PLL_H
#define _FOC_SPEED_PLL_H

#include "foc_header.h"
#include "foc_math.h"
#include "direct_param_pid.hpp"
#include "lowpass_filter.h"

class SpeedPLL
{
public:
    SpeedPLL(speed_pll_t& config) : pi_controller(config.pid_config)
    {
        lpf.Tf = config.Tlp;
    };
    float est_velocity = 0.0f;
    float est_angle = 0.0f;
    float Calculate(float input, float Ts);
    LowpassFilter lpf = LowpassFilter(0.001f);
private:
    DP_PID pi_controller;
};

float SpeedPLL::Calculate(float input, float Ts)
{
    float error1 = arm_sin_f32(input) * arm_cos_f32(est_angle);
    float error2 = arm_cos_f32(input) * arm_sin_f32(est_angle);
    float epsilon = error1 - error2;
    est_velocity = pi_controller.GetOutput(epsilon, Ts);
    est_velocity = lpf.GetOutput(est_velocity, Ts);
    est_angle += est_velocity * Ts;
    est_angle = normalize_rad(est_angle);
    return est_velocity;
}

#endif