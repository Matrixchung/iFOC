#ifndef _UART_PROTOCOL_HPP
#define _UART_PROTOCOL_HPP

#include <cstdint>
#include "utils.h"
#include "string.h"

#define KEEPALIVE_TICKS 500

typedef struct motor_state_t
{
    uint8_t IsAlive;
    uint8_t Enable;
    uint8_t Mode;
    uint8_t MotorTemp;
    uint8_t MCUTemp;
    uint8_t LimiterState;
    uint16_t ErrorCode;
    uint32_t SN;
    float IqReal;
    float IdReal;
    float IqSet;
    float IdSet;
    float Vbus;
    float Ibus;
    float EstimateSpeed;
    float EstimateRawAngle;
    float CurrentLimit;
}motor_state_t;

typedef struct motor_set_t
{
    uint8_t Enable;
    uint8_t Identify;
    uint8_t Mode;
    float IqSet;
    float IdSet;
    float SetSpeed;
    float SetRawAngle;
    float SetTrajectoryAngle;
    float SetCurrentLimit;
}motor_set_t;

class UARTProtocol
{
// private:
public:
    const uint8_t _head[2] = {0x11, 0x45};
public:
    UARTProtocol();
    motor_state_t state;
    motor_set_t set;
    uint8_t state_buffer[sizeof(_head) + sizeof(motor_state_t) + sizeof(uint16_t)];
    uint8_t set_buffer[sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t) + sizeof(uint16_t)]; // added SN identifier
    uint32_t last_rec_tick = 0;
    bool keep_alive = false;
    bool IsAlive();
    void PrepareTxStateBuf();
    void PrepareTxSetBuf();
    bool ReceiveRxStateBuf(uint8_t *buffer, uint8_t len);
    bool ReceiveRxSetBuf(uint8_t *buffer, uint8_t len);
};

bool UARTProtocol::IsAlive()
{
    if(HAL_GetTick() - last_rec_tick >= KEEPALIVE_TICKS) keep_alive = false;
    else keep_alive = true;
    return keep_alive;
}

void UARTProtocol::PrepareTxStateBuf()
{
    memcpy(state_buffer, _head, sizeof(_head));
    memcpy(state_buffer + sizeof(_head), &state, sizeof(motor_state_t));
    uint16_t crc_16 = getCRC16(state_buffer, sizeof(_head) + sizeof(motor_state_t));
    state_buffer[sizeof(_head) + sizeof(motor_state_t)] = (uint8_t)(crc_16 >> 8);
    state_buffer[sizeof(_head) + sizeof(motor_state_t) + 1] = (uint8_t)(crc_16);
}

void UARTProtocol::PrepareTxSetBuf()
{
    memcpy(set_buffer, _head, sizeof(_head));
    // memcpy(set_buffer + sizeof(_head), &set, sizeof(motor_set_t));
    memcpy(set_buffer + sizeof(_head), &state.SN, sizeof(state.SN));
    memcpy(set_buffer + sizeof(_head) + sizeof(state.SN), &set, sizeof(motor_set_t));
    uint16_t crc_16 = getCRC16(set_buffer, sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t));
    set_buffer[sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t)] = (uint8_t)(crc_16 >> 8);
    set_buffer[sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t) + 1] = (uint8_t)(crc_16);
}

bool UARTProtocol::ReceiveRxStateBuf(uint8_t *buffer, uint8_t len)
{
    if(len != sizeof(state_buffer)) return false;
    for(uint8_t i = 0; i < sizeof(_head); i++)
    {
        if(buffer[i] != _head[i]) return false;
    }
    uint16_t crc = (uint16_t)buffer[sizeof(_head) + sizeof(motor_state_t) + 1] | (uint16_t)(buffer[sizeof(_head) + sizeof(motor_state_t)] << 8);
    if(crc != getCRC16(buffer, sizeof(_head) + sizeof(motor_state_t))) return false;
    memcpy(&state, buffer + sizeof(_head), sizeof(motor_state_t));
    last_rec_tick = HAL_GetTick();
    return true;
}

bool UARTProtocol::ReceiveRxSetBuf(uint8_t *buffer, uint8_t len)
{
    if(len != sizeof(set_buffer)) return false;
    for(uint8_t i = 0; i < sizeof(_head); i++)
    {
        if(buffer[i] != _head[i]) return false;
    }
    uint16_t crc = (uint16_t)buffer[sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t) + 1] | (uint16_t)(buffer[sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t)] << 8);
    if(crc != getCRC16(buffer, sizeof(_head) + sizeof(state.SN) + sizeof(motor_set_t))) return false;
    uint32_t SN = 0;
    memcpy(&SN, buffer + sizeof(_head), sizeof(SN));
    if(SN != state.SN) return false;
    memcpy(&set, buffer + sizeof(_head) + sizeof(state.SN), sizeof(motor_set_t));
    last_rec_tick = HAL_GetTick();
    return true;
}

UARTProtocol::UARTProtocol()
{
    memset(&state, 0, sizeof(state));
    memset(&set, 0, sizeof(set));
}

#endif