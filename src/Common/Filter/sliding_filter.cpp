#include "sliding_filter.hpp"

namespace iFOC::Filter
{

SlidingFilter::SlidingFilter(const size_t size)
{
    SetSize(size);
};

real_t SlidingFilter::GetOutput(const real_t input)
{
    if(count < capacity)
    {
        sum += input;
        count++;
    }
    else
    {
        sum += input - old;
    }
    old = input;
    return count == 0 ? 0.0 : sum / (real_t)count;
}

real_t SlidingFilter::GetCurrent() const
{
    return count == 0 ? 0.0 : sum / (real_t)count;
}

void SlidingFilter::Reset()
{
    sum = 0.0f;
    old = 0.0f;
    count = 0;
}

void SlidingFilter::SetSize(const size_t size)
{
    Reset();
    capacity = size;
    if(capacity < 1) capacity = 1;
}

FastSlidingFilter::FastSlidingFilter(size_t size) : Tf(2.0f / ((real_t)size + 1.0f)) {}

real_t FastSlidingFilter::GetOutput(real_t input)
{
    float output = last_input;
    last_input = input;
    output -= Tf * (output - input);
    return output;
};

}