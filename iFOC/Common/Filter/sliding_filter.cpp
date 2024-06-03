#include "sliding_filter.h"
SlidingFilter::SlidingFilter(uint8_t _window_size)
{
    window_size = _constrain(_window_size, 1, MAX_WINDOW_SIZE);
    data_count = 0;
    for(uint8_t i = 0; i < window_size; i++) buffer[i] = 0.0f;
}
SlidingFilter::~SlidingFilter()
{
    for(uint8_t i = 0; i < window_size; i++) buffer[i] = 0.0f;
}
float SlidingFilter::GetOutput(float input)
{
    if(!isWindowFull())
    {
        buffer[data_count++] = input;
        return input;
    }
    float sum = 0.0f;
    memcpy(&buffer[0], &buffer[1], (window_size - 1) * sizeof(float));
    buffer[window_size - 1] = input;
    for(uint8_t i = 0; i < window_size; i++) sum += buffer[i];
    return sum / window_size;
}

bool SlidingFilter::isWindowFull()
{
    return data_count == window_size;
}

void SlidingFilter::Reset()
{
    for(uint8_t i = 0; i < window_size; i++) buffer[i] = 0.0f;
    data_count = 0;
}