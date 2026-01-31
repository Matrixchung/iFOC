#include "hal_const.h"
#include "hal_impl.hpp"
#include "foc_math.hpp"

#if defined(AT32WK_ENV)

#if defined __has_include
#  if __has_include ("arm_math.h") && __has_include ("arm_common_tables.h")
#    include "arm_math.h"
#    include "arm_common_tables.h"
#define ARM_MATH_PRESENT
void our_arm_sin_cos_f32(float32_t theta, float32_t & rSinVal, float32_t & rCosVal);
#  endif
#else
#    include "arm_math.h"
#    include "arm_common_tables.h"
#endif

namespace iFOC::HAL
{
#if defined(ARM_MATH_DSP) && defined(ARM_MATH_PRESENT)
void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos)
{
    // arm_sin_cos_f32(theta, &sin, &cos); // ORIGINAL arm_sin_cos_f32 input is in DEGREE!!!
    our_arm_sin_cos_f32(theta, sin, cos);
}
#else
void sinf_cosf_impl(const real_t theta, real_t& sin, real_t& cos)
{
    sin = std::sinf(theta);
    cos = std::cosf(theta);
}
#endif

uint32_t _get_sn()
{
#if defined(AT32F403Axx) || defined(AT32F407xx)
#define FLASH_SIZE_BASE 0x1FFFF7E0
#define UID_BASE_1      0x1FFFF7E8
#define UID_BASE_2      0x1FFFF7EC
#define UID_BASE_3      0x1FFFF7F0
    const uint8_t *p = (const uint8_t *)UID_BASE_1;
    return ((uint32_t)p[10]<<24) | ((uint32_t)p[1]<<8) | p[0]
        | (((p[9]&0x80)|((p[4]&0x03)<<5)|((p[3]&0xF0)>>3)|(p[2]&0x01))<<16);
#else
#warning "_get_sn() unavailable for unknown chip"
#endif
}

uint32_t GetSerialNumber()
{
    static uint32_t sn = _get_sn();
    return sn;
}

extern "C"
{
    extern const volatile uint32_t __firmware_start;
    extern const volatile uint32_t __firmware_end;
}

uint32_t GetFirmwareSizeBytes()
{
    const uint32_t start_addr = (uint32_t)&__firmware_start;
    const uint32_t end_addr = (uint32_t)&__firmware_end;
    const static uint32_t firmware_size = end_addr - start_addr;
    return firmware_size;
}

uint64_t GetFirmwareCRC64()
{
    static uint64_t crc_result = get_crc64((const uint8_t*)&__firmware_start, GetFirmwareSizeBytes());
    return crc_result;
}
}

#if defined(ARM_MATH_PRESENT)
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

#endif