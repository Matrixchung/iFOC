#pragma once

#include "foc_types.hpp"

namespace iFOC
{
/// PI Controller with trapezoidal integral and back-calculation anti-windup
class PIController
{
public:
    PIController();
    PIController(real_t p, real_t i, real_t _limit, real_t _ramp);
    PIController(float p, float i, float _limit);
    real_t GetOutput(real_t error, real_t Ts, real_t feedforward = 0.0f);
    void Reset();
    void StartMeasureITAE();
    void StopMeasureITAE();
    real_t GetITAE();
    real_t Kp = 0.0f, Ki = 0.0f;
    real_t limit = 0.0f;
    real_t ramp_limit = 0.0f;
private:
    real_t integral = 0.0f;
    real_t back_calc_error_prev = 0.0f;
    real_t output_sat_prev = 0.0f;
    real_t output_prev = 0.0f;
    /// ITAE Measurements
    real_t itae_error_abs_prev = 0.0f;
    real_t itae_sum = 0.0f;
    real_t itae_timer = 0.0f;
    bool itae_measuring = false;
};
}