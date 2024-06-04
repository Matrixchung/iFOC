#ifndef _ENCODER_AB_H
#define _ENCODER_AB_H

#include "global_include.h"
#include "encoder_ab_base.hpp"

class EncoderAB : public EncoderABBase
{
public:
    EncoderAB(TIM_TypeDef *_htim, uint32_t _cpr):
    EncoderABBase(_cpr), htim(_htim) {};
    bool PortInit() override;
    void PortUpdateDirPulse() override;
    void PortSetCounter(uint32_t counter) override { LL_TIM_SetCounter(htim, counter); };
protected:
    TIM_TypeDef *htim;
};

bool EncoderAB::PortInit()
{
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2);
    LL_TIM_DisableIT_BRK(htim);
    LL_TIM_DisableIT_CC1(htim);
    LL_TIM_DisableIT_CC2(htim);
    LL_TIM_DisableIT_CC3(htim);
    LL_TIM_DisableIT_CC4(htim);
    LL_TIM_DisableIT_COM(htim);
    LL_TIM_DisableIT_TRIG(htim);
    LL_TIM_DisableIT_UPDATE(htim);
    LL_TIM_EnableCounter(htim);
    return true;
}

void EncoderAB::PortUpdateDirPulse()
{
    direction = (LL_TIM_GetDirection(htim) == LL_TIM_COUNTERDIRECTION_UP) ? 1 : -1;
    pulse = (short)(LL_TIM_GetCounter(htim));
}

#endif