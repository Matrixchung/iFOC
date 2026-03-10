#pragma once

#include "encoder_off_axis_base.hpp"
#include "../Common/Interface/uart_base.hpp"
#include "../DataType/Ringbuf/kfifo.hpp"

namespace iFOC::Encoder
{
class EncoderOffAxisUART final : public EncoderOffAxisBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(EncoderOffAxisUART);
public:
    explicit EncoderOffAxisUART(HAL::UARTBase* _uart);
    ~EncoderOffAxisUART() override;
    FuncRetCode Init(uint8_t motor_id) override;
    void UpdateMid(float Ts) override;
    float GetChannelA_mV() override;
    float GetChannelB_mV() override;
    bool IsConnected() override;
private:
    static constexpr float CONNECTION_TIMEOUT_SECONDS = 0.5f;
    bool OnRxEvent(uint8_t* data, uint16_t len);
    HAL::UARTBase* uart = nullptr;
    DataType::Ringbuf::kfifo_t rx_fifo;
    float channel_a_mV = 0.0f;
    float channel_b_mV = 0.0f;
    float connection_lost_timer = 0.0f;
    bool connected = false;
    uint8_t rx_handler_id = 0;
};
}