#include "temp_sense_ntc.hpp"

constexpr real_t NOMINAL_TEMP_KELVIN = 25.0f + 273.15f; // 298.15K

namespace iFOC::Sense
{
TempSenseNTC::TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv, real_t ntc_r25, real_t ntc_beta) :
    port(_port), rdiv(_rdiv), ntc_r25_ohm(ntc_r25), ntc_beta_k(ntc_beta)
{
    if(ntc_r25_ohm <= 0.0f) ntc_r25_ohm = 10000.0f;
    if(ntc_beta_k <= 0.0f) ntc_beta_k = 3950.0f;
}

TempSenseNTC::TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv) : port(_port), rdiv(_rdiv) {}

real_t TempSenseNTC::Update()
{
    const auto raw_value = port->GetRawValue();
    if(raw_value == 0)
    {
        temp_celsius = std::numeric_limits<real_t>::max();
        return temp_celsius;
    }
    const real_t ntc_resistance = rdiv * (float)raw_value / (float)(port->GetFullRange() - raw_value);
    constexpr real_t div_nominal = (1.0f / NOMINAL_TEMP_KELVIN);
    const real_t inv_temp_kelvin = div_nominal + std::logf(ntc_resistance / ntc_r25_ohm) / ntc_beta_k;
    temp_celsius = 1.0f / inv_temp_kelvin - 273.15f;
    return temp_celsius;
}
}
