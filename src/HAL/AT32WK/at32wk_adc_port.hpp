#pragma once

#include "../../Common/Interface/adc_port_base.hpp"
#include "../hal_const.h"

#if defined(AT32WK_ENV) && defined(ADC_MODULE_ENABLED)

namespace iFOC::HAL
{
class ADCPort final : public ADCPortBase
{
public:
    ADCPort(uint16_t* _pVal, uint16_t* _pVrefint);
    real_t GetVoltage_mV() override;
    real_t GetFullRangeVoltage() const override;
    uint32_t GetRawValue() override;
    uint32_t GetVrefRawValue() override;
    uint32_t GetFullRange() const override;
private:
    uint16_t* m_pVal;
    uint16_t* m_pVrefint;
};
}

#endif