#pragma once

#include "temp_sense_base.hpp"
#include "../../Common/Interface/adc_port_base.hpp"

namespace iFOC::Sense
{
class TempSenseNTC : public TempSenseBase
{
    DELETE_COPY_CONSTRUCTOR(TempSenseNTC);
public:
    TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv,
        real_t ntc_r25, real_t ntc_beta);
    TempSenseNTC(HAL::ADCPortBase* _port, real_t _rdiv);
    real_t Update() override;
private:
    HAL::ADCPortBase* port;
    real_t rdiv = 10.0f * 1000.0f;
    real_t ntc_r25_ohm = 10000.0f; // 10K NTC
    real_t ntc_beta_k = 3950.0f;
};
}