#ifndef _FOC_FILTER_BASE_H
#define _FOC_FILTER_BASE_H

#include "foc_header.h"

class FilterBase
{
public:
    float GetOutput(float input)
    {
        return 0.0f;
    };
    float GetOutput(float input, float Ts)
    {
        return GetOutput(input);
    };
    virtual void Reset() = 0;
private:

};

#endif