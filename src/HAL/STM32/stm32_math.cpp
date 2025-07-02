#include "hal_impl.hpp"
#include "main.h"

#if defined(USE_HAL_DRIVER)

#if defined __has_include
#  if __has_include ("arm_math.h") && __has_include ("arm_common_tables.h")
#    include "arm_math.h"
#    include "arm_common_tables.h"
void our_arm_sin_cos_f32(float32_t theta, float32_t & rSinVal, float32_t & rCosVal);
#  endif
#else
#    include "arm_math.h"
#    include "arm_common_tables.h"
#endif

// Easily to switch backend of math algorithm

namespace iFOC::HAL
{
// // CORDIC in STM32G4: https://blog.csdn.net/u014319604/article/details/136829630
// #if defined(CORDIC) // slow.
// void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos)
// {
//     CORDIC->CSR = 0x00180061;
//     CORDIC->WDATA = (int32_t)(iFOC::normalize_rad_negPI2posPI(theta) * RADIAN_Q31_f);
//     CORDIC->WDATA = 0x7FFFFFFF;
//     sin = -(float)((int32_t)CORDIC->RDATA) / (float)Q31;
//     cos = -(float)((int32_t)CORDIC->RDATA) / (float)Q31;
// }
#if defined(ARM_MATH_DSP) // currently fastest on STM32G4
void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos)
{
    // arm_sin_cos_f32(theta, &sin, &cos); // ORIGINAL arm_sin_cos_f32 input is in DEGREE!!!
    our_arm_sin_cos_f32(theta, sin, cos);
}
#else // fallback to std
void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos)
{
    sin = std::sinf(theta);
    cos = std::cosf(theta);
}
#endif

uint64_t _get_sn()
{
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = HAL_GetUIDw0();
    uint32_t uuid1 = HAL_GetUIDw1();
    uint32_t uuid2 = HAL_GetUIDw2();
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    return ((uint64_t) uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);
}

uint32_t GetSerialNumber()
{
    static uint64_t sn = _get_sn();
    return (uint32_t)sn;
}

}

void our_arm_sin_cos_f32(float32_t theta, float32_t & rSinVal, float32_t & rCosVal)
{
    float32_t fract, in;
    uint16_t indexS, indexC;   /* Sin/Cos Index Variable */
    float32_t f1, f2, d1, d2;  /* Two nearest output values */
    float32_t Dn, Df;
    float32_t temp, findex;

    /* input x is in radians */
    /* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi */
    in = theta * 0.159154943092f;

    if(in < 0.0f) in = -in;

    in = in - (int32_t)in;

    /* Calculation of index of the table */
    findex = (float32_t)FAST_MATH_TABLE_SIZE * in;
    indexS = ((uint16_t)findex) & 0x1ff;
    indexC = (indexS + (FAST_MATH_TABLE_SIZE / 4)) & 0x1ff;

    /* Calculation of fractional value */
    fract = findex - (float32_t) indexS;

    /* Read two nearest values of input value from the cos & sin tables */
    f1 =  sinTable_f32[indexC  ];
    f2 =  sinTable_f32[indexC+1];
    d1 = -sinTable_f32[indexS  ];
    d2 = -sinTable_f32[indexS+1];

    // rCosVal = (1.0f - fract) * f1 + fract * f2;

    Dn = 0.0122718463030f; /* delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE */
    Df = f2 - f1;          /* delta between the values of the functions */

    temp = Dn * (d1 + d2) - 2 * Df;
    temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
    temp = fract * temp + d1 * Dn;

    /* Calculation of cosine value */
    rCosVal = fract * temp + f1;

    /* Read two nearest values of input value from the cos & sin tables */
    f1 = sinTable_f32[indexS  ];
    f2 = sinTable_f32[indexS+1];
    d1 = sinTable_f32[indexC  ];
    d2 = sinTable_f32[indexC+1];

    // rSinVal = (1.0f - fract) * f1 + fract * f2;

    Df = f2 - f1; // delta between the values of the functions
    temp = Dn * (d1 + d2) - 2 * Df;
    temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
    temp = fract * temp + d1 * Dn;

    /* Calculation of sine value */
    rSinVal = fract * temp + f1;

    if (theta < 0.0f) rSinVal = -rSinVal;
}

#endif