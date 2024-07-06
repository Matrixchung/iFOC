#ifndef _FOC_PID_H
#define _FOC_PID_H

#include "foc_header.h"
#include "foc_math.h"

class PID
{
public:
    PID(float p, float i, float d, float _limit):Kp(p), Ki(i), Kd(d), limit(_limit) {ramp_limit = 0.0f;};
    PID(float p, float i, float d, float _limit, float _ramp):Kp(p), Ki(i), Kd(d), limit(_limit), ramp_limit(_ramp) {};
    PID(pid_config_t config)
    {
        Kp = config.Kp;
        Ki = config.Ki;
        Kd = config.Kd;
        limit = config.limit;
        ramp_limit = config.ramp_limit;
    };
    float GetOutput(float error, float Ts);
    void Reset();
    bool enable_integral = true;
    float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
    float limit = 0.0f;
    float ramp_limit = 0.0f; // positive value
private:
    float error_prev = 0.0f;
    float integral_prev = 0.0f;
    float output_prev = 0.0f;
};

#endif