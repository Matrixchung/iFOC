#include "sliding_filter.hpp"

namespace iFOC::Filter
{
static constexpr size_t MAX_WINDOW_SIZE = 128;

SlidingFilter::SlidingFilter(size_t size) : capacity(_constrain(size, 1, MAX_WINDOW_SIZE)) {};

real_t SlidingFilter::GetOutput(real_t input)
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

void SlidingFilter::Reset()
{
    sum = 0.0f;
    old = 0.0f;
    count = 0;
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