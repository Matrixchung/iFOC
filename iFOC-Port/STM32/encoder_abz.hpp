#ifndef _ENCODER_ABZ_H
#define _ENCODER_ABZ_H

#include "global_include.h"
#include "encoder_abz_base.hpp"

class EncoderABZ : public EncoderABZBase
{
public:
    EncoderABZ(TIM_TypeDef *_htim, uint32_t _cpr):
    EncoderABZBase(_cpr), htim(_htim) {};
    bool PortInit() override;
    void PortUpdateDirPulse() override;
    void PortZeroSignalIRQ() override;
protected:
    TIM_TypeDef *htim;
};

bool EncoderABZ::PortInit()
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
    // LL_TIM_EnableIT_UPDATE(ret.htim);
    LL_TIM_DisableIT_UPDATE(htim);
    LL_TIM_EnableCounter(htim);
    return true;
}

void EncoderABZ::PortUpdateDirPulse()
{
    direction = (LL_TIM_GetDirection(htim) == LL_TIM_COUNTERDIRECTION_UP) ? 1 : -1;
    pulse = (short)(LL_TIM_GetCounter(htim));
}

void EncoderABZ::PortZeroSignalIRQ()
{
    zero_signal = true;
    if(LL_TIM_GetDirection(htim) == LL_TIM_COUNTERDIRECTION_UP)
    {
        LL_TIM_SetCounter(htim, 0);
        full_rotations++;
    }
    else
    {
        LL_TIM_SetCounter(htim, cpr);
        full_rotations--;
    }
}

#endif