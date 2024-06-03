#include "lowpass_filter.h"
float LowpassFilter::GetOutput(float input, float Ts)
{
    if(Ts < 0.0f) Ts = last_dt;
    else if(Ts > 0.3f)  // delta time too long for lowpass filter
    {
        output_prev = input;
        last_dt = Ts;
        return input;
    }
    float alpha = Tf / (Tf + Ts);
    output_prev = alpha * output_prev + (1.0f - alpha) * input;
    last_dt = Ts;
    return output_prev;
}

void LowpassFilter::Reset()
{
    output_prev = 0.0f;
    last_dt = 0.0f;
}