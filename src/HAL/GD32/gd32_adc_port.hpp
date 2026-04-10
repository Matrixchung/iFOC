#pragma once

#include "../../Common/Interface/adc_port_base.hpp"
#include "../hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::HAL
{
class ADCPort final : public ADCPortBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(ADCPort);
public:
    ADCPort(uint32_t* _pVal, uint32_t* _pVrefint);
    [[nodiscard]] real_t GetVoltage_mV() override;
    [[nodiscard]] real_t GetFullRangeVoltage() const override;
    [[nodiscard]] uint32_t GetRawValue() override;
    [[nodiscard]] uint32_t GetVrefRawValue() override;
    [[nodiscard]] uint32_t GetFullRange() const override;
private:
    uint32_t* m_pVal;
    uint32_t* m_pVrefint;
};
}

#endif
