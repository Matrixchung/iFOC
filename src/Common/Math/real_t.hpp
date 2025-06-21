#pragma once

#define USE_STD_MATH

#if (!defined(USE_IQ)) &&  (!defined(USE_STD_MATH))
#define USE_STD_MATH
#endif

//#include "iq_t.hpp"
//#include "iqmath.hpp"
#include "math_defs.hpp"

#ifdef USE_IQ
using real_t = iFOC::iq_t<IQ_DEFAULT_Q>;
#elif defined(USE_DOUBLE)
using real_t = double;
#else
using real_t = float;
#include <cmath>
#endif

namespace iFOC
{

//    static constexpr real_t pi_4 = real_t(PI/4);
//    static constexpr real_t pi_2 = real_t(PI/2);
//    static constexpr real_t pi = real_t(PI);
//    static constexpr real_t tau = real_t(TAU);


    consteval real_t operator"" _r(long double x){
        return real_t(x);
    }

    consteval real_t operator"" _r(unsigned long long x){
        return real_t(x);
    }


    __fast_inline constexpr int mean(const int a, const int b){
        return ((a+b) >> 1);
    }

    template<floating T>
    __fast_inline constexpr T mean(const T & a, const T & b){
        return (a+b) / 2.0f;
    }

    template<typename T>
    __fast_inline constexpr T frac(const T fv){
        return (fv - T(int(fv)));
    }

    template<floating T>
    __fast_inline constexpr T round(const T x)
    {
        return int(x+0.5f);
    }

    template<integral T>
    __fast_inline constexpr T sign(const T val)
    {
        return val == 0 ? 0 : (val < 0 ? -1 : 1);
    }

    template<floating T>
    __fast_inline constexpr T sign(const T fv)
    {
        if(fv > 0.0f) return 1.0f;
        else if(fv < 0.0f) return -1.0f;
        return 0.0f;
    }

}