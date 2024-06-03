#include "speed_pll.h"
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