#include "qpll.hpp"

namespace iFOC::Filter
{
QPLL::QPLL() : QPLL(0.0f, 0.0f, 0.0f) {}

QPLL::QPLL(const float p, const float i, const float _limit) : Kp(p), Ki(i), omega_limit_rad_s(_limit) {}

void QPLL::GetOutput(const float alpha, const float beta, const float Ts)
{
    float pll_sin = 0.0f, pll_cos = 0.0f;
    HAL::sinf_cosf_impl(angle_rad, pll_sin, pll_cos);
    const float y = alpha * pll_sin, z = beta * pll_cos;
    const float error = -y + z;

    float back_calc_error = 0.0f;
    if(Kp != 0.0f) back_calc_error = error - (output_prev - output_sat_prev) / Kp;
    else back_calc_error = error;
    integral += Ki * Ts * 0.5f * (back_calc_error + back_calc_error_prev);
    integral = _constrain(integral, -omega_limit_rad_s, omega_limit_rad_s);
    back_calc_error_prev = back_calc_error;

    const float output = Kp * error + integral;
    output_prev = output;

    const float output_sat = _constrain(output, -omega_limit_rad_s, omega_limit_rad_s);
    output_sat_prev = output_sat;

    omega_rad_s = output_sat;

    angle_rad = normalize_rad(angle_rad + omega_rad_s * Ts);
}

void QPLL::Reset()
{
    angle_rad = 0.0f;
    omega_rad_s = 0.0f;
    integral = 0.0f;
    back_calc_error_prev = 0.0f;
    output_sat_prev = 0.0f;
    output_prev = 0.0f;
}
}
