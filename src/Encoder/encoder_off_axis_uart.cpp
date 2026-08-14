#include "encoder_off_axis_uart.hpp"
#include "../DataType/Headers/Encoder/encoder_off_axis_struct.h"

// static constexpr size_t RX_FIFO_SIZE = iFOC::roundup_pow2(sizeof(encoder_off_axis_struct_t) * 4);
static constexpr size_t RX_FIFO_SIZE = 64;

namespace iFOC::Encoder
{
// EncoderOffAxisUART::EncoderOffAxisUART(HAL::UARTBase* _uart) : uart(_uart)
EncoderOffAxisUART::EncoderOffAxisUART(HAL::UARTHSBase* _uart) : uart(_uart)
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
    // rx_handler_id = uart->RegisterRxHandler(std::bind(&EncoderOffAxisUART::OnRxEvent, this, std::placeholders::_1, std::placeholders::_2));
    rx_handler_id = uart->RegisterRxHandler(std::bind(&EncoderOffAxisUART::OnRxEvent, this, std::placeholders::_1));
    return ret;
}

void EncoderOffAxisUART::UpdateMid(const float Ts)
{
    EncoderOffAxisBase::UpdateMid(Ts);
    uart->UpdateRxFIFO();
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
    return connected && IN_RANGE(GetChannelA_mV(), 200.0f, 3100.0f) && IN_RANGE(GetChannelB_mV(), 200.0f, 3100.0f);
}

void EncoderOffAxisUART::OnRxEvent(HAL::UARTHSBase* uart)
{
    // Consume all in FIFO at once
    if(uart->rx_fifo.used() < 2 * sizeof(encoder_off_axis_struct_t)) return;
    uint8_t byte = 0;
    while(uart->rx_fifo.used())
    {
        uint8_t new_byte;
        uart->rx_fifo.get(&new_byte, 1);
        if(new_byte == 0x5F && byte == 0x70) // treat tail (0x70, 0x5F) as start
        {
            byte = 0;
            if(uart->rx_fifo.used() >= 10)
            {
                uint8_t payload[10];
                uart->rx_fifo.get(payload, 10);
                const uint8_t calc_crc8 = get_crc8(payload, 9);
                if(calc_crc8 == payload[9])
                {
                    connection_lost_timer = 0.0f;
                    flags.reg = payload[0];
                    if(!flags.bit.hw_ready) connected = false;
                    else connected = true;
                    float temp;
                    memcpy(&temp, &payload[1], 4);
                    channel_a_mV = temp;
                    memcpy(&temp, &payload[5], 4);
                    channel_b_mV = temp;
                    return;
                }
                uart->rx_fifo.flush();
            }
        }
        else byte = new_byte;
    }
}

// bool EncoderOffAxisUART::OnRxEvent(uint8_t* data, uint16_t len)
// {
//     do
//     {
//         const uint32_t size_to_put = MIN(rx_fifo.available(), (uint32_t)len);
//         rx_fifo.put(data, size_to_put);
//         len -= size_to_put;
//         data += size_to_put;
//         if(rx_fifo.used() >= sizeof(encoder_off_axis_struct_t))
//         {
//             bool flushed = false;
//             const uint32_t used_size = MIN(rx_fifo.used(), RX_FIFO_SIZE);
//             Vector<uint8_t> buffer(used_size);
//             rx_fifo.peek(buffer.data(), used_size); // peek all buffer first
//             // fifo.wipe_n condition: #1, found tail1 & tail2; #2, fifo full (flush all)
//             if(rx_fifo.available() == 0)
//             {
//                 rx_fifo.flush();
//                 flushed = true;
//             }
//             for(uint32_t i = 0; i < (used_size - 1); i++)
//             {
//                 if(buffer[i] == 0x70 && buffer[i + 1] == 0x5F)
//                 {
//                     const uint32_t total_len = i + 2;
//                     // wipe_n
//                     if(!flushed)
//                     {
//                         rx_fifo.wipe_n(total_len);
//                         flushed = true;
//                     }
//                     if(total_len >= sizeof(encoder_off_axis_struct_t))
//                     {
//                         // processing encoder struct
//                         encoder_off_axis_struct_t temp;
//                         memcpy(&temp, buffer.data() + i - (sizeof(encoder_off_axis_struct_t) - 2 * sizeof(uint8_t)), sizeof(encoder_off_axis_struct_t));
//                         const uint8_t calc_crc8 = get_crc8((const uint8_t*)&temp, sizeof(encoder_off_axis_struct_t) - 3 * sizeof(uint8_t));
//                         if(calc_crc8 == temp.crc8)
//                         {
//                             connection_lost_timer = 0.0f; // reset timer here
//                             flags.reg = temp.flags.reg;
//                             if(!flags.bit.hw_ready) connected = false;
//                             else connected = true;
//                             channel_a_mV = temp.channel_a_mV;
//                             channel_b_mV = temp.channel_b_mV;
//                         }
//                         // else connected = false;
//                         break;
//                     }
//                 }
//             }
//         }
//     } while(len > 0);
//     return false;
// }
}
