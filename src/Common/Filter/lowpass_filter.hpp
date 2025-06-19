#pragma once

#include <cstdint>
#include "foc_math.hpp"

namespace iFOC::Filter
{
// https://blog.csdn.net/wanrenqi/article/details/105559070
class LowpassFilter
{
    OVERRIDE_NEW();
public:
    // explicit LowpassFilter(real_t _Tf) : f_c(1.0f / _Tf) {};
    explicit LowpassFilter(real_t fc) : r_c(1.0f / (PI2 * fc)) {};
    real_t GetOutput(real_t input, real_t Ts);
    void Reset();
    void SetFc(real_t fc) { r_c = 1.0f / (PI2 * fc); };
// private:
    real_t r_c = 1.0f;
    real_t output_prev = 0.0f;
    // real_t last_dt = 0.0f;
};
}