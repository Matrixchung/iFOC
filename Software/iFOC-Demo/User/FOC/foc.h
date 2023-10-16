#ifndef _FOC_H
#define _FOC_H

#include "lowpass_filter.h"

// struct
typedef struct FOC_TypeDef
{
    float zero_electric_angle;
    float voltage_limit;
    float half_voltage_limit;
    float U_alpha, U_beta, U_a, U_b, U_c;
    float dc_a, dc_b, dc_c;
    float velocity;
    LPFilter_TypeDef velocity_filter;
}FOC_TypeDef;

#include "main.h"
#include "DRV8313.h"
#include "user_task.h"
// #include "pwm_encoder.h"
#include "spi_encoder.h"
#include "IQMathLib.h"

// Motor parameters
#define MOTOR_POLE_PAIRS 14
#define MOTOR_ENC_DIR -1 // CW Positive

extern float zero_electric_angle;

// math nums
#define _PI 3.141592653589793238462643383279f
#define _2PI 6.283185307179586476925286766559f
#define _sqrt3 1.7320508075688772935274463415059f
#define _sqrt3_2 0.86602540378443864676372317075294f
#define _sqrt3neg -1.7320508075688772935274463415059f
#define _3PI_2 4.71238898038469f
#define pi_180 0.01745329251994329576923690768489f
#define pi180 57.295779513082320876798154814105f

// macros
#define _constrain(v,low,high) ((v)<(low)?(low):((v)>(high)?(high):(v)))
#define degToRad(deg) ((float)(deg)*pi_180)
#define radToDeg(rad) ((float)(rad)*pi180)
#define FOC_Driver_ResetOutput() DRV8313_ResetPWM()

// inline functions
__STATIC_INLINE float _normalizeRad(float rad)
{
    float a = fmodf(rad, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// void printPhaseVoltage(void);
// void FOC_Driver_Init(void);
FOC_TypeDef FOC_Driver_Init(float voltage_limit);
void FOC_Position_Closeloop(FOC_TypeDef *foc, float motor_target);
float FOC_Velocity_Openloop(FOC_TypeDef *foc, float target_rad);
void FOC_Velocity_Closeloop(FOC_TypeDef *foc, float target_rad);
void FOC_CalcVelocity(FOC_TypeDef *foc);
float FOC_GetElecRad(void);
#endif