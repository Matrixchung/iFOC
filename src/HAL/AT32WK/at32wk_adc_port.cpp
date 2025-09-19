#include "at32wk_adc_port.hpp"

#if defined(AT32WK_ENV) && defined(ADC_MODULE_ENABLED)

constexpr float VintRV = 1.20f;

namespace iFOC::HAL
{
ADCPort::ADCPort(uint16_t* _pVal, uint16_t* _pVrefint) : m_pVal(_pVal), m_pVrefint(_pVrefint) {}

real_t ADCPort::GetVoltage()
{
    // if(!m_pVrefint || *m_pVrefint == 0) return 0.0f;
    if(*m_pVrefint == 0) return 0.0f;
    return (real_t)(VintRV * (real_t)(*m_pVal) / (real_t)(*m_pVrefint));
}

real_t ADCPort::GetFullRangeVoltage()
{
    return 3.3f;
}

uint32_t ADCPort::GetRawValue()
{
    return *m_pVal;
}

uint32_t ADCPort::GetFullRange()
{
#if defined(AT32F403Axx)
    return 4095;
#endif
    return 0;
}
}

#endif
