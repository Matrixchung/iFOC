#include "stm32_temp_sense_core.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)

namespace iFOC::Sense
{
TempSenseCore::TempSenseCore(HAL::ADCPortBase* _port) : port(_port) {}

real_t TempSenseCore::Update()
{
    temp_celsius = 0.0f;
#if defined(LL_ADC_RESOLUTION_16B)

#else defined(LL_ADC_RESOLUTION_12B)
    temp_celsius = (real_t)(__LL_ADC_CALC_TEMPERATURE(__LL_ADC_CALC_VREFANALOG_VOLTAGE(port->GetVrefRawValue(), LL_ADC_RESOLUTION_12B), port->GetRawValue(), LL_ADC_RESOLUTION_12B));
#endif
    return temp_celsius;
}
}

#endif