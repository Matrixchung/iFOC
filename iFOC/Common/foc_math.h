#ifndef _FOC_MATH_H
#define _FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#include "foc_header.h"
#include "math.h"
// Constant defines
#ifndef PI
#define PI 3.141592653589793238462643383279f
#endif
#define _PI_3 1.0471975511965977461542144610932f
#define _3PI_2 4.7123889803846898576939650749193f
#define PI2 6.283185307179586476925286766559f
#define SQRT_3 1.7320508075688772935274463415059f
#define SQRT_3div2 0.86602540378443864676372317075294f
#define divSQRT_3 0.57735026918962576450914878050195f
#define div2SQRT_3 1.15470053838f
// Macro defines
#define DEG2RAD(degree) ((float)(degree) * 0.01745329251994329576923690768489f)
#define RAD2DEG(radian) ((float)(radian) * 57.295779513082320876798154814105f)
#define _constrain(v,low,high) ((v)<(low)?(low):((v)>(high)?(high):(v)))
#define SQ(x) ((x) * (x))
#define SIGN(x)	(((x) < 0.0f) ? -1.0f : 1.0f)
#define NORM2_f(x,y) (sqrtf(SQ(x) + SQ(y)))
#define ABS(x) (((x) < 0.0f) ? -(x) : (x))
// Inline functions
/*
    @brief Normalize radian to [0, 2PI]
    @param radian: any radian
    @retval radian at range of [0, 2PI]
*/
// Function prototypes
alphabeta_t FOC_Clark_ABC(abc_t input);
alphabeta_t FOC_Clark_AB(ab_t input);
qd_t FOC_Park(alphabeta_t input, float theta);
alphabeta_t FOC_Rev_Park(qd_t input, float theta);
// void FOC_SVPWM(alphabeta_t Ualphabeta, svpwm_t *svpwm, float Ubus);
void FOC_SVPWM(svpwm_t *svpwm, float Vbus);
float normalize_rad(float radian);
float fast_atan2f(float y, float x);
float fast_atanf(float x);
float max3(const float *data);
float min3(const float *data);
float max(const float *data, uint16_t len);
float min(const float *data, uint16_t len);
float rad_speed_to_RPM(float omega, uint8_t pole_pair);
float RPM_speed_to_rad(float rpm, uint8_t pole_pair);
float origin_to_shaft(float origin, float gear_ratio);
float shaft_to_origin(float shaft, float gear_ratio);
float sat(float s, float delta);
uint8_t getCRC8(uint8_t* data, uint16_t len);
float fast_inv_sqrt(float x);
float euclid_distance(float x1, float y1, float x2, float y2);
#endif