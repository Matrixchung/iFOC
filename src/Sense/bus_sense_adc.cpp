#include "bus_sense_adc.hpp"
#include "../DataType/board_config.hpp"

#define config iFOC::BoardConfig().GetConfig()

constexpr uint16_t CURRENT_ZERO_OFFSET_STABLE_TIMES = 1000;

namespace iFOC::Sense
{
BusSenseADC::BusSenseADC(HAL::ADCPortBase* _vbus,
                         real_t _vbus_gain,
                         HAL::ADCPortBase* _ibus,
                         real_t _ibus_gain,
                         bool _rev) :
        Vbus_port(_vbus), Ibus_port(_ibus),
        Ibus_zero_lpf(10), Ibus_lpf(config.get_current_sense_f_lp() / 10),
        Vbus_gain_V(_vbus_gain), Ibus_gain_mV(_ibus_gain), reversed(_rev)
{
    // To get Ibus in [A], Ibus_factor_mV should be transformed
    if(config.bus_sense_shunt_ohm() <= 0.0f) Ibus_gain_mV = 0.0f;
    else Ibus_gain_mV = (1.0f / (Ibus_gain_mV * config.get_bus_sense_shunt_ohm() * 1000.0f));
}

BusSenseADC::BusSenseADC(HAL::ADCPortBase* _vbus,
                     real_t _vbus_gain,
                     HAL::ADCPortBase* _ibus,
                     real_t _ibus_gain) : BusSenseADC(_vbus, _vbus_gain, _ibus, _ibus_gain, false) {}

FuncRetCode BusSenseADC::Update() // only read Vbus here
{
    voltage = Vbus_port->GetVoltage() * Vbus_gain_V;
    return FuncRetCode::OK;
}

void BusSenseADC::UpdateRT(float Ts)
{
    if(zero_offset_calc_times < CURRENT_ZERO_OFFSET_STABLE_TIMES)
    {
        current = 0.0f;
        return;
    }
    current = Ibus_lpf.GetOutput((Ibus_port->GetVoltage_mV() - zero_Ibus) * Ibus_gain_mV, Ts);
    if(reversed) current *= -1.0f;
}

void BusSenseADC::UpdateRemainingCurrent(float Ts)
{
    if(zero_offset_calc_times < CURRENT_ZERO_OFFSET_STABLE_TIMES) zero_offset_calc_times++;
    zero_Ibus = Ibus_zero_lpf.GetOutput(Ibus_port->GetVoltage_mV(), Ts);
}
}
