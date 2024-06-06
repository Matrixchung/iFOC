#ifndef _ENCODER_PWM_IT_H
#define _ENCODER_PWM_IT_H

#include "encoder_pwm_base.hpp"
#include "global_include.h"

// PWM Encoder measured by interrupts
// Here we use LPTIM

class EncoderPWM_IT : public EncoderPWMBase
{
public:
    EncoderPWM_IT(LPTIM_HandleTypeDef *_htim, uint32_t exti, float _freq): htim(_htim), exti_line(exti), tim_freq(_freq) {};
protected:
    bool PortTIMInit() override;
    void PortICInterrupt() override;
private:
    LPTIM_HandleTypeDef *htim;
    uint32_t exti_line = LL_EXTI_LINE_0;
    float tim_freq = 25000000.0f;
    float high_time = 0.0f;
    float low_time = 0.0f;
};

bool EncoderPWM_IT::PortTIMInit()
{
    LL_EXTI_DisableIT_0_31(exti_line);
    LL_EXTI_DisableFallingTrig_0_31(exti_line);
    LL_EXTI_EnableRisingTrig_0_31(exti_line);
    HAL_LPTIM_Counter_Start(htim, 0xFFFF);
    LL_EXTI_EnableIT_0_31(exti_line);
    return true;
}

void EncoderPWM_IT::PortICInterrupt()
{
    if(LL_EXTI_IsEnabledRisingTrig_0_31(exti_line)) // entered interrupt with rising trigger, means we are meeting rising edge
    {
        LL_EXTI_DisableRisingTrig_0_31(exti_line);
        LL_EXTI_EnableFallingTrig_0_31(exti_line); // we want falling edge next time
        // we get counter, this is low time
        capture_2 = HAL_LPTIM_ReadCounter(htim);
        // reset counter
        __HAL_LPTIM_RESET_COUNTER(htim);
    }
    else if(LL_EXTI_IsEnabledFallingTrig_0_31(exti_line))
    {
        LL_EXTI_DisableFallingTrig_0_31(exti_line);
        LL_EXTI_EnableRisingTrig_0_31(exti_line); // we want rising edge next time
        capture_1 = HAL_LPTIM_ReadCounter(htim);
        __HAL_LPTIM_RESET_COUNTER(htim);
        if(capture_1 > 0)
        {
            frequency = tim_freq / (float)(capture_1 + capture_2);
            float duty = (float)capture_1 / (float)(capture_1 + capture_2);
            irq_single_round_angle = normalize_rad(PI2 * (duty - ENCODER_PWM_MINIMUM_DUTY) / (ENCODER_PWM_DUTY_SECTION));
            error_flag = 0;
        }
    }
}

#endif