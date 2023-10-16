#ifndef _PWM_ENCODER_H
#define _PWM_ENCODER_H

#include "main.h"
#include "tim.h"
#include "lowpass_filter.h"

#define AS5048A_PWM_FREQUENCY 1000 // Sampling Freq. = 1 MHz (should >= 360 * 1 KHz = 360 KHz)
#define AS5048A_PWM_FREQUENCY_MAX (AS5048A_PWM_FREQUENCY * 1.2f)
#define AS5048A_PWM_FREQUENCY_MIN (AS5048A_PWM_FREQUENCY * 0.8f)
#define AS5048A_INIT_PWM_CLOCKS 16 // First High 16 clocks then 4095 data clocks
#define AS5048A_EXIT_PWM_CLOCKS 8 // End low 8 clocks
#define AS5048A_DIR -1

#define _2PI 6.283185307179586476925286766559f
#define pi180 57.295779513082320876798154814105f
#define ROTATION_CHECK_THRESHOLD (0.8f * _2PI)

extern float encoder_pwm_duty;
extern float encoder_pwm_freq;
extern float encoder_rad;
extern int encoder_full_rotations;

// void PWM_Encoder_Init(TIM_HandleTypeDef *htim);
// extern void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void PWM_Encoder_HardwareInit_LL(TIM_TypeDef *tim);
void PWM_Encoder_SoftwareInit(void);
void PWM_Encoder_IRQHandler_LL(void);
float PWM_Encoder_GetVelocity(void);

// #define PWM_Encoder_GetAngle() ((float)((AS5048A_DIR == 1 ? encoder_pwm_duty : (1.0f-encoder_pwm_duty))* 360.0f))
#define PWM_Encoder_GetRawRad() (encoder_rad)
#define PWM_Encoder_GetRawAngle() ((float)(encoder_rad * pi180))
#define PWM_Encoder_GetAngle() ((float)(encoder_full_rotations * 360.0f) + (float)(encoder_rad * pi180))
#define PWM_Encoder_GetRad() ((float)(encoder_full_rotations * _2PI) + encoder_rad)
// #define PWM_Encoder_GetRad() ((float)((AS5048A_DIR == 1 ? encoder_pwm_duty : (1.0f-encoder_pwm_duty)) * _2PI))

#endif