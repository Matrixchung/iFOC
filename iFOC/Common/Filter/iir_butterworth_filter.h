#ifndef _FOC_IIR_BUTTERWORTH_FILTER
#define _FOC_IIR_BUTTERWORTH_FILTER

#include "lowpass_filter.h"
// Using MATLAB filterDesigner to define constants
// Structure: Direct-Form II, Second-Order Sections
// Numerator[0:2] -> b[0:2]
// Denominator[0:2] -> a[0:2]
// Gain -> input_gain
// Output Gain -> output_gain
class IIRFilter : public FilterBase
{
public:
    IIRFilter(float _b0, float _b1, float _b2, float _a0, float _a1, float _a2, float _input_gain, float _output_gain)
    :b0(_b0), b1(_b1), b2(_b2), a0(_a0), a1(_a1), a2(_a2), input_gain(_input_gain), output_gain(_output_gain)
    {
        w0 = 0.0f;
        w1 = 0.0f;
    };
    float GetOutput(float input);
    void Reset();
private:
    float w0, w1; // w(n-2), w(n-1)
    float b0, b1, b2;
    float a0, a1, a2;
    float input_gain;
    float output_gain;
};

#endif