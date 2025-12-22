#include "temp_sense_ntc.hpp"

constexpr real_t A = 0.0008736528f;
constexpr real_t B = 0.000253893f;
constexpr real_t C = 0.0000001816f;

namespace iFOC::Sense
{
TempSenseNTC::TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv) : port(_port), rdiv(_rdiv) {}

real_t TempSenseNTC::Update()
{
    auto raw_value = port->GetRawValue();
    if(raw_value == 0)
    {
        temp_celsius = std::numeric_limits<real_t>::max();
        return temp_celsius;
    }
    real_t ntc_resistance = rdiv / (((float)port->GetFullRange() / (float)raw_value) - 1.0f);
    real_t ntc_Ln = std::logf(ntc_resistance);
    temp_celsius = (1.0f / (A + B * ntc_Ln + C * ntc_Ln * ntc_Ln * ntc_Ln)) - 273.15f;
    return temp_celsius;
}
}
