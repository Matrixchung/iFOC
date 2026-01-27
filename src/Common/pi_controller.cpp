#include "pi_controller.hpp"

namespace iFOC
{
PIController::PIController() : PIController(0.0f, 0.0f, 0.0f) {}
PIController::PIController(const real_t p, const real_t i, const real_t _limit, const real_t _ramp) : Kp(p), Ki(i), limit(_limit), ramp_limit(_ramp) {}
PIController::PIController(const float p, const float i, const float _limit) : PIController(p, i, _limit, 0.0f) {}

real_t PIController::GetOutput(const real_t error, const real_t Ts, const real_t feedforward)
{
    /// ITAE Measurement
    if(itae_measuring)
    {
        itae_timer += Ts;
        real_t error_abs = std::abs(error);
        itae_sum += itae_timer * Ts * 0.5f * (error_abs + itae_error_abs_prev);
        itae_error_abs_prev = error_abs;
    }

    real_t back_calc_error = 0.0f;
    if(Kp != 0.0f) back_calc_error = error - (output_prev - output_sat_prev) / Kp;
    else back_calc_error = error;
    integral += Ki * Ts * 0.5f * (back_calc_error + back_calc_error_prev);
    integral = _constrain(integral, -limit, limit);
    back_calc_error_prev = back_calc_error;

    const real_t output = Kp * error + integral + feedforward;
    output_prev = output;

    real_t output_sat = 0.0f;
    if(ramp_limit > 0.0f)
    {
        const real_t max_delta = ramp_limit * Ts;
        real_t delta = output - output_sat_prev;
        delta = _constrain(delta, -max_delta, max_delta);
        output_sat = output_sat_prev + delta;
    }
    else output_sat = output;
    output_sat = _constrain(output_sat, -limit, limit);
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

real_t PIController::GetITAE() const
{
    return itae_sum;
}
}