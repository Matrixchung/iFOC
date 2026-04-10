#pragma once

#include "encoder_off_axis_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"

namespace iFOC::Encoder
{
class EncoderOffAxisADC final : public EncoderOffAxisBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(EncoderOffAxisADC);
public:
    EncoderOffAxisADC(HAL::ADCPortBase* _ch_a, HAL::ADCPortBase* _ch_b);
    float GetChannelA_mV() override;
    float GetChannelB_mV() override;
    bool IsConnected() override;
private:
    HAL::ADCPortBase* ch_a;
    HAL::ADCPortBase* ch_b;
};
}