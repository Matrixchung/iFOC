#include "pid.h"
float PID::GetOutput(float error, float Ts)
{
    float integral = 0.0f;
    float output = Kp * error;
    if(enable_integral)
    {
        integral = _constrain(integral_prev + (Ki * Ts * 0.5f * (error + error_prev)), -limit - output, limit - output);
        integral_prev = integral;
    }
    else integral_prev = 0.0f;
    output += integral + (Kd * (error - error_prev) / Ts);
    // float output = (Kp * error) + integral + (Kd * (error - error_prev) / Ts);
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

void PID::Reset()
{
    error_prev = 0.0f;
    integral_prev = 0.0f;
}

void PID::EnableIntegral()
{
    enable_integral = true;
}

void PID::DisableIntegral()
{
    enable_integral = false;
}