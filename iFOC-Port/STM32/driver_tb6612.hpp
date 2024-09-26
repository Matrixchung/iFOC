#ifndef _DRIVER_TB6612_H
#define _DRIVER_TB6612_H

#include "driver_bdc_base.hpp"
#include "global_include.h"

class DriverTB6612 : public DriverBDCBase<DriverTB6612>
{
public:
    DriverTB6612(TIM_TypeDef *_htim, uint32_t channel, GPIO_TypeDef *_in1_port, uint32_t _in1_pin, GPIO_TypeDef *_in2_port, uint32_t _in2_pin):
    DriverBDCBase(LL_TIM_GetAutoReload(_htim)), htim(_htim), tim_channel(channel), in1_port(_in1_port), in1_pin(_in1_pin), in2_port(_in2_port), in2_pin(_in2_pin) {};
    bool Init(bool initCNT);
    void SetOutputRaw(uint16_t ch_1, uint8_t dir);
    void EnableOutput() {LL_TIM_EnableAllOutputs(htim);};
    void DisableOutput() {LL_TIM_DisableAllOutputs(htim);};
    TIM_TypeDef *htim;
private:
    typedef void (*_set_compare)(TIM_TypeDef *TIMx, uint32_t compare);
    _set_compare set_cmp;
    uint32_t tim_channel = LL_TIM_CHANNEL_CH1;
    GPIO_TypeDef *in1_port;
    uint32_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint32_t in2_pin;
};

bool DriverTB6612::Init(bool initCNT)
{
    LL_GPIO_ResetOutputPin(in1_port, in1_pin);
    LL_GPIO_ResetOutputPin(in2_port, in2_pin);
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

void DriverTB6612::SetOutputRaw(uint16_t ch_1, uint8_t dir)
{
    set_cmp(htim, ch_1);
    if(dir) 
    {
        LL_GPIO_SetOutputPin(in1_port, in1_pin);
        LL_GPIO_ResetOutputPin(in2_port, in2_pin);
    }
    else 
    {
        LL_GPIO_ResetOutputPin(in1_port, in1_pin);
        LL_GPIO_SetOutputPin(in2_port, in2_pin);
    }
}

#endif