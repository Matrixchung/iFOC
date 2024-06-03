#ifndef _BUS_SENSE_H
#define _BUS_SENSE_H

#include "bus_sense_base.hpp"
#include "global_include.h"

class BusSense : public BusSenseBase<BusSense>
{
public:
    BusSense(uint16_t& _Vref, uint16_t& _Vbus): Vrefint(_Vref), Vbus_raw(_Vbus) {};
    void Update();
private:
    uint16_t& Vrefint;
    uint16_t& Vbus_raw;
    SlidingFilter vbus_filter = SlidingFilter(20);
};

void BusSense::Update()
{
    Ibus = 0.0f;
    Vbus = 0.001f * 11.0f * (float)(__LL_ADC_CALC_DATA_TO_VOLTAGE(__LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_12B), Vbus_raw, LL_ADC_RESOLUTION_12B));
    Vbus = vbus_filter.GetOutput(Vbus);
}

#endif