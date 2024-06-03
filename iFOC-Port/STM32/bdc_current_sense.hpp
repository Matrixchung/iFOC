#ifndef _BRUSHED_CURRENT_SENSE_H
#define _BRUSHED_CURRENT_SENSE_H

#include "current_sense_base.hpp"
#include "global_include.h"

class BDCCurrentSense : public CurrentSenseBase<BDCCurrentSense>
{
public:
    BDCCurrentSense(float sense_gain, float shunt_mohm, uint16_t& _Vrefint, uint16_t& _Vsense)
    : CurrentSenseBase(sense_gain, shunt_mohm), Vrefint(_Vrefint), Vsense(_Vsense) {};
    void Update();
private:
    uint16_t& Vrefint;
    uint16_t& Vsense;
};

void BDCCurrentSense::Update()
{
    uint32_t _vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_16B);
    // Iabc.a = ((float)__LL_ADC_CALC_DATA_TO_VOLTAGE(_vref, (uint32_t)Vsense, LL_ADC_RESOLUTION_16B)) - ((float)_vref * 0.5f);
    Iabc.a = ((float)__LL_ADC_CALC_DATA_TO_VOLTAGE(_vref, (uint32_t)Vsense, LL_ADC_RESOLUTION_16B)); // one side
    Iabc.a *= current_factor;
}

#endif