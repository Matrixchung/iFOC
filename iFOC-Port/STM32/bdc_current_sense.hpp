#ifndef _BRUSHED_CURRENT_SENSE_H
#define _BRUSHED_CURRENT_SENSE_H

#include "current_sense_base.hpp"
#include "lowpass_filter.h"
#include "global_include.h"

class BDCCurrentSense : public CurrentSenseBase<BDCCurrentSense>
{
public:
    BDCCurrentSense(float sense_gain, float shunt_mohm, uint16_t& _Vrefint, uint16_t& _Vsense)
    : CurrentSenseBase(sense_gain, shunt_mohm), Vrefint(_Vrefint), Vsense(_Vsense) {};
    void Update();
private:
    LowpassFilter lp = LowpassFilter(0.01f);
    uint16_t& Vrefint;
    uint16_t& Vsense;
};

void BDCCurrentSense::Update()
{
    Iabc.a = ((float)Vsense * (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_16B) / 65535.0f);
    Iabc.a *= current_factor;
    Iabc.a = lp.GetOutput(Iabc.a, 0.0001f);
}

#endif