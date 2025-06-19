#pragma once

#include <cstdint>
#include "foc_math.hpp"

namespace iFOC::Filter
{
class SlidingFilter
{
    OVERRIDE_NEW();
public:
    explicit SlidingFilter(size_t size);
    // SlidingFilter(const SlidingFilter&) = delete;
    // SlidingFilter& operator=(const SlidingFilter&) = delete;
    real_t GetOutput(real_t input);
    void Reset();
private:
    real_t sum = 0.0f;
    real_t old = 0.0f;
    size_t capacity = 0;
    size_t count = 0;
};

class FastSlidingFilter
{
    OVERRIDE_NEW();
public:
    explicit FastSlidingFilter(size_t size);
    real_t GetOutput(real_t input);
    void Reset() {};
private:
    real_t Tf = 1.0f;
    real_t last_input = 0.0f;
};
}