#ifndef _FOC_SPEED_PLL_H
#define _FOC_SPEED_PLL_H

#include "foc_header.h"
#include "foc_math.h"
#include "pid.h"
#include "lowpass_filter.h"

class SpeedPLL
{
public:
    SpeedPLL(float p ,float i, float _limit)
    {
        Kp = p;
        Ki = i;
        limit = _limit;
        pi_controller = PID(Kp, Ki, 0.0f, limit);
    };
    float est_velocity = 0.0f;
    float est_angle = 0.0f;
    float Calculate(float input, float Ts);
private:
    float Kp = 0.0f, Ki = 0.0f, limit = 0.0f;
    PID pi_controller = PID(0.0f, 0.0f, 0.0f, 0.0f);
    LowpassFilter lpf = LowpassFilter(0.001f);
};

#endif