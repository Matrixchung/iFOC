#include "lowpass_filter.hpp"

namespace iFOC::Filter
{
real_t LowpassFilter::GetOutput(real_t input, real_t Ts)
{
    // // if(Ts < 0.0f) Ts = last_dt;
    real_t alpha = r_c / (r_c + Ts);
    output_prev = alpha * output_prev + (1.0f - alpha) * input;
    // // last_dt = Ts;
    return output_prev;
}

void LowpassFilter::Reset()
{
    output_prev = 0.0f;
    // last_dt = 0.0f;
}

}