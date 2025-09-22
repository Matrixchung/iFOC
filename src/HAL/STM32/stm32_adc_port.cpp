#include "stm32_adc_port.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)

namespace iFOC::HAL
{
ADCPort::ADCPort(uint16_t* _pVal, uint16_t* _pVrefint) : m_pVal(_pVal), m_pVrefint(_pVrefint) {}

real_t ADCPort::GetVoltage_mV()
{
    if(*m_pVrefint == 0) return 0.0f;
    real_t Vref_mV = 0.0f;
#if defined(LL_ADC_RESOLUTION_16B)
    Vref_mV = (real_t)(__LL_ADC_CALC_VREFANALOG_VOLTAGE(*m_pVrefint, LL_ADC_RESOLUTION_16B));
#elif defined(LL_ADC_RESOLUTION_12B)
    Vref_mV = (real_t)(__LL_ADC_CALC_VREFANALOG_VOLTAGE(*m_pVrefint, LL_ADC_RESOLUTION_12B));
#endif
    return Vref_mV * (real_t)GetRawValue() / (real_t)GetFullRange();
}

real_t ADCPort::GetFullRangeVoltage() const
{
    return 3.3f;
}

uint32_t ADCPort::GetRawValue()
{
    return *m_pVal;
}

uint32_t ADCPort::GetVrefRawValue()
{
    return *m_pVrefint;
}

uint32_t ADCPort::GetFullRange() const
{
#if defined(LL_ADC_RESOLUTION_16B)
    return 65535;
#elif defined(LL_ADC_RESOLUTION_12B)
    return 4095;
#endif
    return 0;
}
}

#endif