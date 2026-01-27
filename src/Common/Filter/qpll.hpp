#pragma once

#include "../foc_math.hpp"

// https://zhuanlan.zhihu.com/p/588141511

namespace iFOC::Filter
{
class QPLL
{
    OVERRIDE_NEW();
public:
    QPLL();
    QPLL(float p, float i, float _limit);
    void GetOutput(float alpha, float beta, float Ts);
    void Reset();
    float Kp = 0.0f, Ki = 0.0f;
    float omega_limit_rad_s = 0.0f;
    float angle_rad = 0.0f;
    float omega_rad_s = 0.0f;
private:
    float integral = 0.0f;
    float back_calc_error_prev = 0.0f;
    float output_sat_prev = 0.0f;
    float output_prev = 0.0f;
};
}