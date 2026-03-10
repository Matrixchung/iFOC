#include "encoder_off_axis_uart.hpp"
#include "../DataType/Headers/Encoder/encoder_off_axis_struct.h"

// static constexpr size_t RX_FIFO_SIZE = iFOC::roundup_pow2(sizeof(encoder_off_axis_struct_t) * 2);
static constexpr size_t RX_FIFO_SIZE = 32;

namespace iFOC::Encoder
{
EncoderOffAxisUART::EncoderOffAxisUART(HAL::UARTBase* _uart) : uart(_uart)
{
    static_assert(sizeof(encoder_off_axis_struct_t) > 1); // we should make sure that buffer doesnt overflow
    static_assert(sizeof(encoder_off_axis_struct_t) > 3 * sizeof(uint8_t));
}

EncoderOffAxisUART::~EncoderOffAxisUART()
{
    uart->RemoveRxHandler(rx_handler_id);
}

FuncRetCode EncoderOffAxisUART::Init(uint8_t motor_id)
{
    auto ret = EncoderOffAxisBase::Init(motor_id);
    rx_fifo.init(RX_FIFO_SIZE);
    rx_handler_id = uart->RegisterRxHandler(std::bind(&EncoderOffAxisUART::OnRxEvent, this, std::placeholders::_1, std::placeholders::_2));
    return ret;
}

void EncoderOffAxisUART::UpdateMid(const float Ts)
{
    EncoderOffAxisBase::UpdateMid(Ts);
    connection_lost_timer += Ts;
    if(connection_lost_timer >= CONNECTION_TIMEOUT_SECONDS) connected = false;
}

float EncoderOffAxisUART::GetChannelA_mV()
{
    return channel_a_mV;
}

float EncoderOffAxisUART::GetChannelB_mV()
{
    return channel_b_mV;
}

bool EncoderOffAxisUART::IsConnected()
{
    return connected;
}

bool EncoderOffAxisUART::OnRxEvent(uint8_t* data, uint16_t len)
{
    do
    {
        const uint32_t size_to_put = MIN(rx_fifo.available(), (uint32_t)len);
        rx_fifo.put(data, size_to_put);
        len -= size_to_put;
        data += size_to_put;
        if(rx_fifo.used() >= sizeof(encoder_off_axis_struct_t))
        {
            bool flushed = false;
            const uint32_t used_size = MIN(rx_fifo.used(), RX_FIFO_SIZE);
            Vector<uint8_t> buffer(used_size);
            rx_fifo.peek(buffer.data(), used_size); // peek all buffer first
            // fifo.wipe_n condition: #1, found tail1 & tail2; #2, fifo full (flush all)
            if(rx_fifo.available() == 0)
            {
                rx_fifo.flush();
                flushed = true;
            }
            for(uint32_t i = 0; i < (used_size - 1); i++)
            {
                if(buffer[i] == 0x70 && buffer[i + 1] == 0x5F)
                {
                    const uint32_t total_len = i + 2;
                    // wipe_n
                    if(!flushed)
                    {
                        rx_fifo.wipe_n(total_len);
                        flushed = true;
                    }
                    if(total_len >= sizeof(encoder_off_axis_struct_t))
                    {
                        // processing encoder struct
                        encoder_off_axis_struct_t temp{};
                        memcpy(&temp, buffer.data() + i - (sizeof(encoder_off_axis_struct_t) - 2 * sizeof(uint8_t)), sizeof(encoder_off_axis_struct_t));
                        const uint8_t calc_crc8 = get_crc8((const uint8_t*)&temp, sizeof(encoder_off_axis_struct_t) - 3 * sizeof(uint8_t));
                        if(calc_crc8 == temp.crc8)
                        {
                            connection_lost_timer = 0.0f; // reset timer here
                            flags.reg = temp.flags.reg;
                            if(!flags.bit.hw_ready) connected = false;
                            else connected = true;
                            channel_a_mV = temp.channel_a_mV;
                            channel_b_mV = temp.channel_b_mV;
                        }
                        // else connected = false;
                        break;
                    }
                }
            }
        }
    } while(len > 0);
    return false;
}
}
