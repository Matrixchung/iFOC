#include "iir_butterworth_filter.h"
float IIRFilter::GetOutput(float input)
{
    float w2 = 0.0f;
    float output = 0.0f;
    w2 = (a0 * input_gain * input) - (a1 * w1) - (a2 * w0);
    output = output_gain * ((b0 * w2) + (b1 * w1) + (b2 * w0));
    w0 = w1;
    w1 = w2;
    return output;
}

void IIRFilter::Reset()
{
    w0 = 0.0f;
    w1 = 0.0f;
}