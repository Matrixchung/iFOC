#include "bus_sense_adc.hpp"
#include "../DataType/board_config.hpp"

// Theory: https://mp.weixin.qq.com/s/UnFQhpdq9zFAkN5XJdbicg

#define config iFOC::BoardConfig().GetConfig()

namespace iFOC::Sense
{
BusSenseADC::BusSenseADC(HAL::ADCPortBase* _vbus,
                         const real_t _vbus_gain,
                         HAL::ADCPortBase* _ibus,
                         const real_t _ibus_gain,
                         const bool _rev) :
        Vbus_port(_vbus), Ibus_port(_ibus),
        Ibus_lpf(100), Ibus_offset_lpf(10),
        Vbus_gain_V(_vbus_gain), Ibus_gain_mV(_ibus_gain)
{
    if(_ibus && Ibus_port)
    {
        // To get Ibus in [A], Ibus_factor_mV should be transformed
        if(config.bus_sense_shunt_ohm() <= 0.0f) Ibus_gain_mV = 0.0f;
        else Ibus_gain_mV = (1.0f / (Ibus_gain_mV * config.get_bus_sense_shunt_ohm() * 1000.0f));
        Ibus_offset_lpf.output_prev = Ibus_port->GetFullRangeVoltage() * 1000.0f * 0.5f;
        if(_rev) Ibus_gain_mV *= -1.0f;
    }
}

BusSenseADC::BusSenseADC(HAL::ADCPortBase* _vbus,
                     const real_t _vbus_gain,
                     HAL::ADCPortBase* _ibus,
                     const real_t _ibus_gain) : BusSenseADC(_vbus, _vbus_gain, _ibus, _ibus_gain, false) {}

BusSenseADC::BusSenseADC(HAL::ADCPortBase* _vbus, const real_t _vbus_gain) : BusSenseADC(_vbus, _vbus_gain, nullptr, 1.0f, false) {}

FuncRetCode BusSenseADC::Update()
{
    voltage = Vbus_port->GetVoltage() * Vbus_gain_V;
    if(Ibus_port)
    {
        const auto curr_tick = xTaskGetTickCount();
        if(last_update_tick > 0)
        {
            const float Ts = (curr_tick - last_update_tick) * 0.001f;
            current = Ibus_lpf.GetOutput((Ibus_port->GetVoltage_mV() - Ibus_offset_lpf.output_prev) * Ibus_gain_mV, Ts);
        }
        last_update_tick = curr_tick;
    }
    else current = 0.0f;
    return FuncRetCode::OK;
}

void BusSenseADC::SampleDCOffset(uint16_t sample_ms)
{
    if(!Ibus_port) return;
    if(sample_ms == 0)
    {
        Ibus_offset_lpf.GetOutput(Ibus_port->GetVoltage_mV(), 0.0001f); // sample once
        return;
    }
    sample_ms = _constrain(sample_ms, 1, 100);
    for(uint16_t i = 0; i < sample_ms * 10; i++)
    {
        Ibus_offset_lpf.GetOutput(Ibus_port->GetVoltage_mV(), 0.0001f);
        HAL::DelayUs(100);
    }
}

// void BusSenseADC::UpdateRT(float Ts)
// {
//     if(zero_offset_calc_times < CURRENT_ZERO_OFFSET_STABLE_TIMES)
//     {
//         current = 0.0f;
//         return;
//     }
//     current = Ibus_sf.GetOutput((Ibus_port->GetVoltage_mV() - 1650.0f) * Ibus_gain_mV);
//     if(reversed) current *= -1.0f;
// }
//
// void BusSenseADC::UpdateRemainingCurrent(float Ts)
// {
//     if(zero_offset_calc_times < CURRENT_ZERO_OFFSET_STABLE_TIMES) zero_offset_calc_times++;
//     zero_Ibus = Ibus_zero_lpf.GetOutput(Ibus_port->GetVoltage_mV(), Ts);
// }
}
