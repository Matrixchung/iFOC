#pragma once

#include "bus_sense_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"
#include "../Common/Filter/lowpass_filter.hpp"

namespace iFOC::Sense
{
class BusSenseADC final : public BusSenseBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(BusSenseADC);
public:
    BusSenseADC() = delete;
    BusSenseADC(HAL::ADCPortBase* _vbus, real_t _vbus_gain,
                HAL::ADCPortBase* _ibus, real_t _ibus_gain, bool _rev);
    BusSenseADC(HAL::ADCPortBase* _vbus, real_t _vbus_gain,
                HAL::ADCPortBase* _ibus, real_t _ibus_gain);
    FuncRetCode Update() override;
    void SampleDCOffset(uint16_t sample_ms);
private:
    HAL::ADCPortBase* Vbus_port;
    HAL::ADCPortBase* Ibus_port;
    Filter::LowpassFilter Ibus_lpf;
    Filter::LowpassFilter Ibus_offset_lpf;
    real_t Vbus_gain_V = 1.0f;
    real_t Ibus_gain_mV = 1.0f;
    TickType_t last_update_tick = 0;
    bool reversed = false;
};
}