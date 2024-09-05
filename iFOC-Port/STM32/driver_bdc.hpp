#ifndef _DRIVER_BDC_H
#define _DRIVER_BDC_H

#include "driver_bdc_base.hpp"
#include "global_include.h"

class DriverBDC : public DriverBDCBase<DriverBDC>
{
public:
    DriverBDC(TIM_TypeDef *_htim, uint32_t channel, GPIO_TypeDef *_ph_port, uint32_t _ph_pin):
    DriverBDCBase(LL_TIM_GetAutoReload(_htim)), htim(_htim), tim_channel(channel), ph_port(_ph_port), ph_pin(_ph_pin) {};
    bool Init(bool initCNT);
    void SetOutputRaw(uint16_t ch_1, uint8_t dir);
    void EnableOutput() {LL_TIM_EnableAllOutputs(htim);};
    void DisableOutput() {LL_TIM_DisableAllOutputs(htim);};
    TIM_TypeDef *htim;
private:
    typedef void (*_set_compare)(TIM_TypeDef *TIMx, uint32_t compare);
    _set_compare set_cmp;
    uint32_t tim_channel = LL_TIM_CHANNEL_CH1;
    GPIO_TypeDef *ph_port;
    uint32_t ph_pin;
};

bool DriverBDC::Init(bool initCNT)
{
    LL_GPIO_ResetOutputPin(ph_port, ph_pin);
    switch(tim_channel)
    {
        case LL_TIM_CHANNEL_CH2:
            set_cmp = LL_TIM_OC_SetCompareCH2;
            break;
        case LL_TIM_CHANNEL_CH3:
            set_cmp = LL_TIM_OC_SetCompareCH3;
            break;
        default:
            set_cmp = LL_TIM_OC_SetCompareCH1;
            break;
    }   
    LL_TIM_CC_EnableChannel(htim, tim_channel);
    __attribute__((unused)) uint32_t tmpsmcr = htim->SMCR & TIM_SMCR_SMS;
    max_compare = LL_TIM_GetAutoReload(htim);
    if(initCNT)
    {
        EnableOutput();
        LL_TIM_EnableCounter(htim);
    }
    return true;
}

void DriverBDC::SetOutputRaw(uint16_t ch_1, uint8_t dir)
{
    set_cmp(htim, ch_1);
    if(dir) LL_GPIO_SetOutputPin(ph_port, ph_pin);
    else LL_GPIO_ResetOutputPin(ph_port, ph_pin); // Fast Decay provided by hardware design
}

#endif