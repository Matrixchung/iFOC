#include "pi_controller.hpp"

namespace iFOC
{
PIController::PIController() : PIController(0.0f, 0.0f, 0.0f) {}
PIController::PIController(real_t p, real_t i, real_t _limit, real_t _ramp) : Kp(p), Ki(i), limit(_limit), ramp_limit(_ramp) {}
PIController::PIController(float p, float i, float _limit) : PIController(p, i, _limit, 0.0f) {}

real_t PIController::GetOutput(real_t error, real_t Ts, real_t feedforward)
{
    /// ITAE Measurement
    if(itae_measuring)
    {
        itae_timer += Ts;
        float error_abs = std::abs(error);
        itae_sum += itae_timer * Ts * 0.5f * (error_abs + itae_error_abs_prev);
        itae_error_abs_prev = error_abs;
    }

    float back_calc_error = 0.0f;
    if(Kp != 0.0f) back_calc_error = error - (output_prev - output_sat_prev) / Kp;
    integral += Ki * Ts * 0.5f * (back_calc_error + back_calc_error_prev);
    back_calc_error_prev = back_calc_error;

    float output = Kp * error + integral + feedforward;
    output_prev = output;

    float output_sat = _constrain(output, -limit, limit);
    if(ramp_limit > 0.0f)
    {
        float output_rate = (output_sat - output_sat_prev) / Ts;
        if(output_rate > ramp_limit) output_sat = output_sat_prev + ramp_limit * Ts;
        else if(output_rate < -ramp_limit) output_sat = output_sat_prev - ramp_limit * Ts;
    }
    output_sat_prev = output_sat;
    return output_sat;
}

void PIController::Reset()
{
    integral = 0.0f;
    back_calc_error_prev = 0.0f;
    output_sat_prev = 0.0f;
    output_prev = 0.0f;
}

void PIController::StartMeasureITAE()
{
    itae_error_abs_prev = 0.0f;
    itae_sum = 0.0f;
    itae_timer = 0.0f;
    itae_measuring = true;
}

void PIController::StopMeasureITAE()
{
    itae_measuring = false;
}

real_t PIController::GetITAE()
{
    return itae_sum;
}
}