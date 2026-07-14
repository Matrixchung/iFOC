#include "at32wk_temp_sense_core.hpp"

#if defined(AT32WK_ENV) && defined(ADC_MODULE_ENABLED)

#if defined(AT32F403AxG)
constexpr real_t V25 = 1.28f;
constexpr real_t Avg_Slope_mV_C = -4.26f;
#elif defined(AT32F435xx)
constexpr real_t V25 = 1.27f;;
constexpr real_t Avg_Slope_mV_C = -4.13f;
#endif

namespace iFOC::Sense
{
TempSenseCore::TempSenseCore(HAL::ADCPortBase* _port) : port(_port) {}

real_t TempSenseCore::Update()
{
    temp_celsius = (((V25 - port->GetVoltage()) * 1000.0f) / Avg_Slope_mV_C) + 25.0f;
    return temp_celsius;
}
}

#endif