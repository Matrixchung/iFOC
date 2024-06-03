#ifndef _FOC_SLIDING_FILTER_H
#define _FOC_SLIDING_FILTER_H

#include "filter_base.h"
#include "foc_math.h"
#include <string.h>

#define MAX_WINDOW_SIZE 51

class SlidingFilter : public FilterBase
{
public:
    explicit SlidingFilter(uint8_t _window_size);
    ~SlidingFilter();
    float GetOutput(float input);
    bool isWindowFull();
    void Reset();
private:
    uint8_t window_size;
    uint8_t data_count;
    float buffer[MAX_WINDOW_SIZE];
};

#endif