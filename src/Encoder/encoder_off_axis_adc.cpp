#include "encoder_off_axis_adc.hpp"

namespace iFOC::Encoder
{

EncoderOffAxisADC::EncoderOffAxisADC(HAL::ADCPortBase* _ch_a, HAL::ADCPortBase* _ch_b) : ch_a(_ch_a), ch_b(_ch_b) {}

float EncoderOffAxisADC::GetChannelA_mV()
{
    return ch_a->GetVoltage_mV();
}

float EncoderOffAxisADC::GetChannelB_mV()
{
    return ch_b->GetVoltage_mV();
}

bool EncoderOffAxisADC::IsConnected()
{
    return IN_RANGE(GetChannelA_mV(), 200.0f, 3100.0f) && IN_RANGE(GetChannelB_mV(), 200.0f, 3100.0f);
}
}
