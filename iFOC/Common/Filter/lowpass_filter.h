#ifndef _FOC_LOWPASS_FILTER_H
#define _FOC_LOWPASS_FILTER_H

#include "filter_base.h"

class LowpassFilter : public FilterBase
{
public:
    explicit LowpassFilter(float time_constant) : Tf(time_constant) {};
    float GetOutput(float input, float Ts);
    void Reset();
    float Tf;
private:
    float output_prev;
    float last_dt;
};

#endif