#pragma once

#include "foc_types.hpp"
#include "hal_impl.hpp"
#include "Math/math_concepts.hpp"

namespace iFOC::HAL
{
static void DelayUs(volatile uint32_t us)
{
    DelayCycle(us * (GetCoreClockHz() / 1000000));
}
static void DelayMs(uint32_t ms)
{
    while(ms--) DelayUs(1000);
}
}

namespace iFOC
{
/// Clarke equal-amplitude transformation
/// \param abc Three-shunt currents
/// \return
static alphabeta_t FOC_Clark(const std::array<real_t, 3>& abc)
{
    return {
        .alpha = ((2.0f * abc[0]) - abc[1] - abc[2]) * 0.333333333f,
        .beta = (abc[1] - abc[2]) * divSQRT_3
    };
}

static qd_t FOC_Park(const alphabeta_t input, const real_t theta)
{
    real_t _sin, _cos;
    HAL::sinf_cosf_impl(theta, _sin, _cos);
    return {
        .q = _cos * input.beta - _sin * input.alpha,
        .d = _cos * input.alpha + _sin * input.beta
    };
}

static alphabeta_t FOC_Rev_Park(const qd_t input, const real_t theta)
{
    real_t _sin, _cos;
    HAL::sinf_cosf_impl(theta, _sin, _cos);
    return {
        .alpha = _cos * input.d - _sin * input.q,
        .beta = _cos * input.q + _sin * input.d
    };
}
/// Midpoint-clamped SVPWM modulation with qd-axis voltage limitation applied\n
/// Reference: https://zhuanlan.zhihu.com/p/663825561
/// \param Uqd_real Uq and Ud, [V]
/// \param theta_rad Electric angle, [rad]
/// \param Udc_real Bus voltage Udc, [V]
/// \param Tabc Three-shunt open time, per-unit, range: [0, 1]
static void FOC_SVPWM(qd_t Uqd_real, const real_t theta_rad, const real_t Udc_real, std::array<real_t, 3>& Tabc)
{
    // Step #1: Limit Uq, to ensure maximum Ud and compound vector also under modulation circle
    real_t Udc_divSQRT3 = Udc_real * divSQRT_3;
    Uqd_real.d = _constrain(Uqd_real.d, -Udc_divSQRT3, Udc_divSQRT3);
    real_t Uq_limit = std::sqrtf(Udc_divSQRT3 * Udc_divSQRT3 - Uqd_real.d * Uqd_real.d);
    Uqd_real.q = _constrain(Uqd_real.q, -Uq_limit, Uq_limit);

    // Step #2: Reversed Park Transform, to get Ualpha and Ubeta
    alphabeta_t Ualphabeta = FOC_Rev_Park(Uqd_real, theta_rad);
    Ualphabeta.alpha /= Udc_real; // get per unit value
    Ualphabeta.beta /= Udc_real;

    // Step #3: Get Uabc
    std::array<real_t, 3> Uabc_Pu = {Ualphabeta.alpha, 0.0f, 0.0f};

    Ualphabeta.alpha *= -0.5f;
    Ualphabeta.beta *= SQRT_3div2;

    Uabc_Pu[1] = Ualphabeta.alpha + Ualphabeta.beta;
    Uabc_Pu[2] = Ualphabeta.alpha - Ualphabeta.beta;

    // Step #4: Midpoint Clamp
    real_t Ucom_Pu = 0.5f * (MAX(Uabc_Pu[0], Uabc_Pu[1], Uabc_Pu[2]) + MIN(Uabc_Pu[0], Uabc_Pu[1], Uabc_Pu[2]));

    // Step #5: Calculate Tabc
    Ucom_Pu -= 0.5f;
    Tabc[0] = Uabc_Pu[0] - Ucom_Pu;
    Tabc[1] = Uabc_Pu[1] - Ucom_Pu;
    Tabc[2] = Uabc_Pu[2] - Ucom_Pu;
}

static real_t fast_atanf(const real_t x)
{
    using namespace _const_tables;
    unsigned short idx = 0;
    real_t atan_val;
    if (x > 1)
    {
        idx = 512 * (x - 1) / (x + 1);
        atan_val = atanTab[idx] + 0.785398f;
    }
    else if (x >= 0 && x <= 1)
    {
        idx = 512 * x;
        atan_val = atanTab[idx];
    }
    else if (x >= -1 && x < 0)
    {
        idx = 512 * (-x);
        atan_val = -atanTab[idx];
    }
    else
    {
        idx = 512 * (-x - 1) / (-x + 1);
        atan_val = -atanTab[idx] - 0.785398f;
    }
    return atan_val;
}

/// Get CRC8 result of data[len] \n
/// Poly: X^8 + X^2 + X + 1 (0x07) \n
/// Initial Value: 0x00, Input/Output Flip: No \n
/// XOR Value: 0x00
/// \param data data array that need to be calculated
/// \param len length of the *data array
/// \return CRC8 value
static uint8_t get_crc8(const uint8_t* data, const size_t len)
{
    using namespace _const_tables;
    uint8_t crc = 0x0;
    for (size_t i = 0; i < len; i++)
    {
        crc = crc8_table[(uint8_t)crc ^ *data];
        data++;
    }
    return crc;
}

/// Get CRC16 result of data[len] \n
/// Poly: X^16 + X^15 + X^2 + 1 (0x8005), CRC-16-IBM(CRC-16/MODBUS) \n
/// Initial Value: 0xFFFF, Input Flip: Yes, Output Flip: No \n
/// XOR Value: 0x0000
/// \param data
/// \param len
/// \return
static uint16_t get_crc16(const uint8_t* data, const size_t len)
{
    using namespace _const_tables;
    uint8_t crc_h = 0xFF;
    uint8_t crc_l = 0xFF;
    uint8_t crc_table_index = 0;
    for(size_t i = 0; i < len; i++)
    {
        crc_table_index = crc_l ^ (*(data++));
        crc_l = (uint8_t)(crc_h ^ crc16_table_h[crc_table_index]);
        crc_h = crc16_table_l[crc_table_index];
    }
    return (uint16_t)(crc_h << 8 | crc_l);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
__fast_inline static float fast_inv_sqrt(const float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
#pragma GCC diagnostic pop

/// Normalize radian to [0, 2PI]
/// \param rad
/// \return [0, 2PI]
__fast_inline static constexpr real_t normalize_rad(const real_t rad)
{
    real_t a = std::fmodf(rad, PI2);
    return a >= 0 ? a : (a + PI2);
}

__fast_inline static constexpr real_t normalize_rad_negPI2posPI(const real_t rad)
{
    return normalize_rad(rad) - PI;
}

__fast_inline static constexpr real_t DEG2RAD(real_t x)
{
    constexpr real_t _2PI_div_360 = 0.01745329251994329576923690768489f;
    return (x * _2PI_div_360);
}

__fast_inline static constexpr real_t RAD2DEG(real_t x)
{
    constexpr real_t _360_div_2PI = 57.295779513082320876798154814105f;
    return (x * _360_div_2PI);
}

__fast_inline static constexpr real_t RAD2RPM(const real_t rad, const uint8_t pole_pair)
{
    constexpr real_t _div30_PI = 9.5492965855137201461330258023509;
    return (real_t)(rad * _div30_PI / (real_t)pole_pair);
}

__fast_inline static constexpr real_t RAD2REV(const real_t rad)
{
    constexpr real_t _div_PI2 = 0.15915494309189533576888376337251;
    return (real_t)(rad * _div_PI2);
}

__fast_inline static constexpr real_t REV2RAD(const real_t rev)
{
    return (real_t)(rev * PI2);
}

__fast_inline static constexpr real_t RPM2RAD(const real_t rpm, const uint8_t pole_pair)
{
    constexpr real_t _divPI_30 = 0.1047197551196597746154214461093;
    return (real_t)(rpm * _divPI_30 * (real_t)pole_pair);
}

__fast_inline static constexpr real_t ELEC2OUTPUT(const real_t elec, const real_t gear_ratio)
{
    return (real_t)(elec / gear_ratio);
}

__fast_inline static constexpr real_t OUTPUT2ELEC(const real_t output, const real_t gear_ratio)
{
    return (real_t)(output * gear_ratio);
}

__fast_inline static constexpr real_t euclid_distance(const real_t x1,
                                                      const real_t y1,
                                                      const real_t x2,
                                                      const real_t y2)
{
    return (real_t)(std::hypotf(x2 - x1, y2 - y1));
}

__fast_inline static constexpr bool check_parity_u16(uint16_t x)
{
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (~x) & 1;
}

__fast_inline uint32_t roundup_pow2(uint32_t data)
{
    return (1UL << (32UL - (uint32_t)__builtin_clz(data)));
}

template< class Enum >
constexpr std::underlying_type_t<Enum> to_underlying( Enum e ) noexcept
{
    return static_cast<std::underlying_type_t<Enum>>(e);
}

template<unsigned long long E, int N>
struct qpow
{
    enum { value = E * qpow<E, N - 1>::value };
};

template <unsigned long long E>
struct qpow<E, 0>
{
    enum { value = 1 };
};

template<int E>
static unsigned long long quick_pow(unsigned int n)
{
    constexpr static unsigned long long lookupTable[] =
    {
        qpow<E, 0>::value, qpow<E, 1>::value, qpow<E, 2>::value,
        qpow<E, 3>::value, qpow<E, 4>::value, qpow<E, 5>::value,
        qpow<E, 6>::value, qpow<E, 7>::value, qpow<E, 8>::value,
        qpow<E, 9>::value, qpow<E, 10>::value, qpow<E, 11>::value,
        qpow<E, 12>::value, qpow<E, 13>::value, qpow<E, 14>::value,
        qpow<E, 15>::value, qpow<E, 16>::value
    };

    return lookupTable[n];
}

template<int E>
static float quick_powf(unsigned int n)
{
    constexpr static float lookupTable[] =
    {
        (float)qpow<E, 0>::value, (float)qpow<E, 1>::value, (float)qpow<E, 2>::value,
        (float)qpow<E, 3>::value, (float)qpow<E, 4>::value, (float)qpow<E, 5>::value,
        (float)qpow<E, 6>::value, (float)qpow<E, 7>::value, (float)qpow<E, 8>::value,
        (float)qpow<E, 9>::value, (float)qpow<E, 10>::value, (float)qpow<E, 11>::value,
        (float)qpow<E, 12>::value, (float)qpow<E, 13>::value, (float)qpow<E, 14>::value,
        (float)qpow<E, 15>::value, (float)qpow<E, 16>::value
    };

    return lookupTable[n];
}

}