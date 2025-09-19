#pragma once

#include "temp_sense_base.hpp"
#include "../../Common/Interface/adc_port_base.hpp"

namespace iFOC::Sense
{
class TempSenseNTC : public TempSenseBase
{
public:
    TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv);
    real_t Update() override;
private:
    HAL::ADCPortBase* port;
    real_t rdiv = 10.0f * 1000.0f;
};
}