#ifndef _FOC_SPEED_PLL_H
#define _FOC_SPEED_PLL_H

#include "foc_header.h"
#include "foc_math.h"
#include "pid.h"
#include "lowpass_filter.h"

class SpeedPLL
{
public:
    SpeedPLL(float p ,float i, float limit, float Tlp)
    {
        pi_controller.Kp = p;
        pi_controller.Ki = i;
        pi_controller.limit = limit;
        lpf.Tf = Tlp;
    };
    SpeedPLL() {};
    float est_velocity = 0.0f;
    float est_angle = 0.0f;
    float Calculate(float input, float Ts);
    PID pi_controller = PID(0.0f, 0.0f, 0.0f, 0.0f);
    LowpassFilter lpf = LowpassFilter(0.001f);
};

#endif