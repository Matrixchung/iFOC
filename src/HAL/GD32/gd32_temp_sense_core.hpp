#pragma once

#include "temp_sense_base.hpp"
#include "../../Common/Interface/adc_port_base.hpp"
#include "../hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::Sense
{
class TempSenseCore final : public TempSenseBase
{
    OVERRIDE_NEW();
public:
    explicit TempSenseCore(HAL::ADCPortBase* _port);
    real_t Update() override;
private:
    HAL::ADCPortBase* port;
};
}

#endif