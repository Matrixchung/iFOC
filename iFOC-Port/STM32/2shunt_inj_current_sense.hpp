#ifndef _TWO_SHUNT_INJECTED_CURRENT_SENSE_H
#define _TWO_SHUNT_INJECTED_CURRENT_SENSE_H

#include "current_sense_base.hpp"
#include "global_include.h"

class TwoShuntCurrentSense : public CurrentSenseBase<TwoShuntCurrentSense>
{
public:
    TwoShuntCurrentSense(float sense_gain, float shunt_mohm, ADC_TypeDef *_hadc, uint16_t& _Vrefint)
    : CurrentSenseBase(sense_gain, shunt_mohm), hadc(_hadc), Vrefint(_Vrefint){};
    void Update();
    ADC_TypeDef *hadc;
private:
    SlidingFilter filter_1 = SlidingFilter(5);
    SlidingFilter filter_2 = SlidingFilter(5);
    uint16_t& Vrefint;
};

void TwoShuntCurrentSense::Update()
{
    // uint32_t _vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_12B);
    // Iabc.a = ((float)__LL_ADC_CALC_DATA_TO_VOLTAGE(_vref, LL_ADC_INJ_ReadConversionData32(hadc, LL_ADC_INJ_RANK_1), LL_ADC_RESOLUTION_12B)) - ((float)_vref * 0.5f);
    // Iabc.b = ((float)__LL_ADC_CALC_DATA_TO_VOLTAGE(_vref, LL_ADC_INJ_ReadConversionData32(hadc, LL_ADC_INJ_RANK_2), LL_ADC_RESOLUTION_12B)) - ((float)_vref * 0.5f);
    // Iabc.c = ((float)__LL_ADC_CALC_DATA_TO_VOLTAGE(_vref, LL_ADC_INJ_ReadConversionData32(hadc, LL_ADC_INJ_RANK_3), LL_ADC_RESOLUTION_12B)) - ((float)_vref * 0.5f);
    float _vref = (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_12B);
    Iabc.a = (((float)LL_ADC_INJ_ReadConversionData32(hadc, LL_ADC_INJ_RANK_1)) * _vref / 4095.0f) - (_vref * 0.5f);
    Iabc.b = (((float)LL_ADC_INJ_ReadConversionData32(hadc, LL_ADC_INJ_RANK_2)) * _vref / 4095.0f) - (_vref * 0.5f);
    Iabc.a *= current_factor;
    Iabc.b *= current_factor;
    Iabc.a = filter_1.GetOutput(Iabc.a);
    Iabc.b = filter_2.GetOutput(Iabc.b);
    // Iabc.c *= current_factor;
    Iabc.c = -(Iabc.a + Iabc.b);
}

#endif