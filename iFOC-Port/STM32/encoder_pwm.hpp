#ifndef _ENCODER_PWM_H
#define _ENCODER_PWM_H

#include "encoder_pwm_base.hpp"
#include "global_include.h"

class EncoderPWM : public EncoderPWMBase
{
public:
    EncoderPWM(TIM_TypeDef *_htim, float _freq): htim(_htim), tim_freq(_freq) {};
protected:
    bool PortTIMInit() override;
    void PortICInterrupt() override;
private:
    TIM_TypeDef *htim;
    float tim_freq = 1.0f;
};

bool EncoderPWM::PortTIMInit()
{
    if(tim_freq <= 1.0f) tim_freq = 1.0f;
    LL_TIM_EnableIT_CC1(htim);
    // LL_TIM_EnableIT_CC2(htim); // maybe we can disable it?
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(htim);
    return true;   
}

void EncoderPWM::PortICInterrupt()
{
    if(LL_TIM_IsActiveFlag_CC1(htim))
    {
        capture_1 = LL_TIM_IC_GetCaptureCH1(htim);
        capture_2 = LL_TIM_IC_GetCaptureCH2(htim);
        frequency = tim_freq / (float)capture_1;
        if(capture_1 == 0 || frequency > 1200.0f || frequency < 800.0f) error_flag = 1;
        else
        {
            float duty = (float)capture_2 / (float)capture_1;
            // if(duty < ENCODER_PWM_MINIMUM_DUTY) error_flag = 2;
            // else if(duty <= ENCODER_PWM_DUTY_SECTION + ENCODER_PWM_MINIMUM_DUTY)
            // if(duty <= ENCODER_PWM_DUTY_SECTION + ENCODER_PWM_MINIMUM_DUTY)
            // {
                irq_single_round_angle = normalize_rad(PI2 * (duty - ENCODER_PWM_MINIMUM_DUTY) / (ENCODER_PWM_DUTY_SECTION));
                error_flag = 0;
            // }
        }
        LL_TIM_SetCounter(htim, 0);
        LL_TIM_ClearFlag_CC1(htim);
    }
    else
    {
        LL_TIM_ClearFlag_CC2(htim);
    }
}

#endif