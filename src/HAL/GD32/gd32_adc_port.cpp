#include "gd32_adc_port.hpp"

#if defined(GD32_ENV)

#if defined(GD32G5X3)
constexpr real_t VintRV_mV = 1200.0f;
#else
#error "ADC Vrefint not defined for current platform"
#endif

namespace iFOC::HAL
{
ADCPort::ADCPort(uint32_t* _pVal, uint32_t* _pVrefint) : m_pVal(_pVal), m_pVrefint(_pVrefint) {}

real_t ADCPort::GetVoltage_mV()
{
    if(*m_pVrefint == 0) return 0.0f;
    return (VintRV_mV * (real_t)(*m_pVal) / (real_t)(*m_pVrefint));
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
#if defined(GD32G5X3)
    return 4095;
#endif
    return 0;
}
}

#endif