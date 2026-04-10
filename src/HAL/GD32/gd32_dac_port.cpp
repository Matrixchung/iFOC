#include "gd32_dac_port.hpp"

#if defined(GD32_ENV)

#if defined(GD32G5X3)
constexpr real_t VintRV_mV = 1200.0f;
#else
#error "ADC Vrefint not defined for current platform"
#endif

namespace iFOC::HAL
{
DACPort::DACPort(const uint32_t _dac, const uint8_t _ch, uint32_t* _pVrefint) : hdac(_dac), ch(_ch), m_pVrefint(_pVrefint) {}

DACPort::DACPort(const uint32_t _dac, const uint8_t _ch) : DACPort(_dac, _ch, nullptr) {}

void DACPort::Init()
{
    dac_mode_config(hdac, ch, NORMAL_PERIPH_BUFFOFF);
    dac_data_format_config(hdac, ch, DAC_DATA_FORMAT_UNSIGNED);
    dac_wave_mode_config(hdac, ch, DAC_WAVE_DISABLE);
    dac_reset_persist_disable(hdac, ch);
    dac_trigger_disable(hdac, ch);
    dac_trimming_disable(hdac, ch);
    dac_enable(hdac, ch);
    dac_data_set(hdac, ch, DAC_ALIGN_12B_R, 0);
}

void DACPort::SetOutput_mV(real_t voltage)
{
    voltage = _constrain(voltage, 0.0f, 3300.0f);
    uint16_t target = 0;
    if(m_pVrefint && *m_pVrefint > 0)
    {
        target = (uint16_t)((real_t)(*m_pVrefint) * voltage / VintRV_mV);
    }
    else target = (uint16_t)((real_t)(voltage / 3300.0f) * 4095.0f);
    dac_data_set(hdac, ch, DAC_ALIGN_12B_R, target);
}
}

#endif