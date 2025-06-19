#include "foc_harmonic_regulator.hpp"
#include "foc_math.hpp"

namespace iFOC::FOC
{
HarmonicRegulator::HarmonicRegulator(float _kr, float _wc, float _limit)  : Kr(_kr), wc(_wc), output_limit(_limit) {};

float HarmonicRegulator::GetOutput(float error, float w0, float phase_comp_rad, float Ts)
{
    if(std::abs(w0) <= 0.00001f)
    {
        y_1 = 0.0f;
        y_2 = 0.0f;
        return 0.0f;
    }
    if(max_step_current >= 0.00001f)
    {
        if(std::abs(error) >= max_step_current)
        {
            // error = 0.0f;
            error_1 = 0.0f;
            error_2 = 0.0f;
            y_1 = 0.0f;
            y_2 = 0.0f;
            return 0.0f;
        }
    }
    // float y = 0.0f;
    float _w0_Ts = w0 * Ts;
    float _sin_w0_Ts, _cos_w0_Ts;
    HAL::sinf_cosf_impl(_w0_Ts, _sin_w0_Ts, _cos_w0_Ts);
    float _sin_comp, _cos_comp;
    HAL::sinf_cosf_impl(phase_comp_rad, _sin_comp, _cos_comp);

    float _sin_w0_Ts_cos_comp = _sin_w0_Ts * _cos_comp;
    float _1_sub_cos_w0_Ts_sin_comp = (1.0f - _cos_w0_Ts) * _sin_comp;

    float b0 = _sin_w0_Ts_cos_comp - _1_sub_cos_w0_Ts_sin_comp;
    float b1 = -2.0f * _1_sub_cos_w0_Ts_sin_comp;
    float b2 = -_sin_w0_Ts_cos_comp - _1_sub_cos_w0_Ts_sin_comp;
    float a0 = 1.0f + wc / w0 * _sin_w0_Ts;
    float a1 = -2.0f * _cos_w0_Ts;
    float a2 = 2.0f - a0;

    float wc_div_2_w0_a0 = wc / (2.0f * w0 * a0);
    float y = Kr * (b0 * wc_div_2_w0_a0 * error + b1 * wc_div_2_w0_a0 * error_1 + b2 * wc_div_2_w0_a0 * error_2) - a1 / a0 * y_1 - a2 / a0 * y_2;

    y = _constrain(y, -output_limit, output_limit);

    error_2 = error_1;
    error_1 = error;

    y_2 = y_1;
    y_1 = y;

    return y;
}

float HarmonicRegulator::GetOutputWithSinCos(float error, float w0, float sin_w0_Ts, float cos_w0_Ts, float sin_comp, float cos_comp, float Ts)
{
    if(std::abs(w0) <= 0.00001f)
    {
        y_1 = 0.0f;
        y_2 = 0.0f;
        return 0.0f;
    }
    if(max_step_current >= 0.00001f)
    {
        if(std::abs(error) >= max_step_current)
        {
            // error = 0.0f;
            error_1 = 0.0f;
            error_2 = 0.0f;
            y_1 = 0.0f;
            y_2 = 0.0f;
            return 0.0f;
        }
    }
    // float y = 0.0f;

    float _sin_w0_Ts_cos_comp = sin_w0_Ts * cos_comp;
    float _1_sub_cos_w0_Ts_sin_comp = (1.0f - cos_w0_Ts) * sin_comp;

    float b0 = _sin_w0_Ts_cos_comp - _1_sub_cos_w0_Ts_sin_comp;
    float b1 = -2.0f * _1_sub_cos_w0_Ts_sin_comp;
    float b2 = -_sin_w0_Ts_cos_comp - _1_sub_cos_w0_Ts_sin_comp;
    float a0 = 1.0f + wc / w0 * sin_w0_Ts;
    float a1 = -2.0f * cos_w0_Ts;
    float a2 = 2.0f - a0;

    float wc_div_2_w0_a0 = wc / (2.0f * w0 * a0);
    float y = Kr * (b0 * wc_div_2_w0_a0 * error + b1 * wc_div_2_w0_a0 * error_1 + b2 * wc_div_2_w0_a0 * error_2) - a1 / a0 * y_1 - a2 / a0 * y_2;

    y = _constrain(y, -output_limit, output_limit);

    error_2 = error_1;
    error_1 = error;

    y_2 = y_1;
    y_1 = y;

    return y;
}

void HarmonicRegulator::Reset()
{
    error_1 = 0.0f;
    error_2 = 0.0f;
    y_1 = 0.0f;
    y_2 = 0.0f;
}

}