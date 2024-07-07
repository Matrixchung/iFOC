#ifndef _DRIVER_3PWM_H
#define _DRIVER_3PWM_H

#include "driver_pwm_base.hpp"
#include "global_include.h"

class Driver3PWM : public DriverPWMBase<Driver3PWM>
{
public:
    explicit Driver3PWM(TIM_TypeDef *_htim) : htim(_htim) {};
    bool PortTIMInit(bool initCNT);
    void PortSetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3);
    void EnableOutput() {LL_TIM_EnableAllOutputs(htim);};
    void DisableOutput() {LL_TIM_DisableAllOutputs(htim);}; // route noises cause motor unexpectedly spin
    // void DisableOutput() {};
    void SetLSIdleState(uint8_t state) {};
    TIM_TypeDef *htim;
// private:
//     TIM_TypeDef *htim;
};

bool Driver3PWM::PortTIMInit(bool initCNT)
{
    max_compare = LL_TIM_GetAutoReload(htim);
    half_compare = max_compare / 2u;
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH4);
    __attribute__((unused)) uint32_t tmpsmcr = htim->SMCR & TIM_SMCR_SMS;
    if(initCNT)
    {
        EnableOutput();
        LL_TIM_EnableCounter(htim);
    }
    return true;
}

void Driver3PWM::PortSetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3)
{
    LL_TIM_OC_SetCompareCH1(htim, ch_1);
    LL_TIM_OC_SetCompareCH2(htim, ch_2);
    LL_TIM_OC_SetCompareCH3(htim, ch_3);
}

#endif