/**
 * @file direct_param_pid.hpp
 * @author Matrixchung (xin_zhong@std.uestc.edu.cn)
 * @brief a simple PID class which doesn't store arguments copy, but takes references from other places.
 *        used in connecting config variables with inner variables. If you modified config, it will take effects immediately.
 * @version 0.1
 * @date 2024-11-03
 * 
 * Copyright © 2024, iFOC Project
 * 
 */
#ifndef _FOC_DIRECT_PARAM_PID_HPP
#define _FOC_DIRECT_PARAM_PID_HPP

#include "pid.h"

// Pass external reference& directly as param, making it easier to sync with FOC main config.
// Could only be initialized from pid_config_t

class DP_PID
{
public:
    DP_PID(pid_config_t& config)
    : Kp(config.Kp), Ki(config.Ki), Kd(config.Kd), limit(config.limit), ramp_limit(config.ramp_limit) {};
    float GetOutput(float error, float Ts);
    void Reset()
    {
        error_prev = 0.0f;
        integral_prev = 0.0f;
    }
    bool enable_integral = true;
private:
    float& Kp;
    float& Ki;
    float& Kd;
    float& limit;
    float& ramp_limit;
    float error_prev = 0.0f;
    float integral_prev = 0.0f;
    float output_prev = 0.0f;
};

float DP_PID::GetOutput(float error, float Ts)
{
    float integral = 0.0f;
    float output = Kp * error;
    output = _constrain(output, -limit, limit);
    if(Ki != 0.0f && enable_integral)
    {
        integral = _constrain(integral_prev + (Ki * Ts * 0.5f * (error + error_prev)), -limit - output, limit - output);
        integral_prev = integral;
    }
    else integral_prev = 0.0f;
    output += integral + (Kd * (error - error_prev) / Ts);
    output = _constrain(output, -limit, limit);
    if(ramp_limit > 0.0f)
    {
        float output_rate = (output - output_prev) / Ts;
        if(output_rate > ramp_limit) output = output_prev + ramp_limit * Ts;
        else if(output_rate < -ramp_limit) output = output_prev - ramp_limit * Ts;
    }
    output_prev = output;
    error_prev = error;
    return output;
}

#endif