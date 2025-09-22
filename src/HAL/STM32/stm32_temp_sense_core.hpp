#pragma once

#include "temp_sense_base.hpp"
#include "adc_port_base.hpp"
#include "hal_const.h"

#if defined(HAL_ADC_MODULE_ENABLED)

namespace iFOC::Sense
{
class TempSenseCore final : public TempSenseBase
{
public:
    explicit TempSenseCore(HAL::ADCPortBase* _port);
    real_t Update() override;
private:
    HAL::ADCPortBase* port;
};
}

#endif