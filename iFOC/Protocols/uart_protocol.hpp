#ifndef _UART_PROTOCOL_HPP
#define _UART_PROTOCOL_HPP

#include <cstdint>
#include "utils.h"
#include "string.h"
#include "sys/param.h"

class motor_state_t
{
public:
    uint8_t IsAlive = 0;
    uint8_t Enable = 0;
    uint8_t NodeID = 0;
    uint8_t Mode = 0;
    uint8_t MotorTemp = 0;
    uint8_t MCUTemp = 0;
    uint8_t LimiterState = 0;
    uint16_t ErrorCode = 0;
    uint32_t SN = 0;
    float IqReal = 0.0f;
    float IdReal = 0.0f;
    float Vbus = 0.0f;
    float Ibus = 0.0f;
    float EstimateSpeed = 0.0f;
    float EstimateRawAngle = 0.0f;
    float CurrentLimit = 0.0f;
    bool operator==(const motor_state_t& rhs) const { return memcmp(&IsAlive, &rhs.IsAlive, sizeof(motor_state_t)) == 0; }
    bool operator!=(const motor_state_t& rhs) const { return !(*this == rhs); }
};

class motor_set_t
{
public:
    uint8_t Enable = 0;
    uint8_t Mode = 0;
    float IqSet = 0.0f;
    float SetSpeed = 0.0f;
    float SetRawAngle = 0.0f;
    float SetTrajectoryAngle = 0.0f;
    float SetCurrentLimit = 0.0f;
    bool operator==(const motor_set_t& rhs) const { return memcmp(&Enable, &rhs.Enable, sizeof(motor_set_t)) == 0; }
    bool operator!=(const motor_set_t& rhs) const { return !(*this == rhs); }
};

#define PDO_PAYLOAD_LEN 42
#define MOTOR_STATE_HEADER 0x43
#define PAYLOAD_HEADER 0x57
#define UART_TIMEOUT_TICKS 500
#define UART_KEEPALIVE_TICKS 100

class state_buffer_t // upstream to Interface, SN packed in payload
{
public:
    uint8_t header = 0;
    uint8_t crc16[2] = {0};
    uint8_t __attribute__((aligned(4))) payload[MAX(sizeof(motor_state_t), PDO_PAYLOAD_LEN)] = {0};
    bool operator==(const state_buffer_t& rhs) const { return memcmp(&header, &rhs.header, sizeof(state_buffer_t)) == 0; }
    bool operator!=(const state_buffer_t& rhs) const { return !(*this == rhs); }
    bool isValid()
    {
        if(header != MOTOR_STATE_HEADER && header != PAYLOAD_HEADER) return false;
        uint16_t crc_16 = 0;
        memcpy(&crc_16, crc16, 2);
        if(crc_16 == getCRC16(payload, sizeof(payload))) return true;
        return false;
    }
    void prepare()
    {
        uint16_t crc_16 = getCRC16(payload, sizeof(payload));
        memcpy(crc16, &crc_16, 2);
    }
    [[nodiscard]] motor_state_t * state() const { return (motor_state_t*)payload; }
};

class set_buffer_t  // Interface downstream to Driver, need SN
{
public:
    uint8_t header = 0;
    uint8_t crc16[2] = {0};
    uint8_t SN[4] = {0};
    uint8_t __attribute__((aligned(4))) payload[MAX(sizeof(motor_set_t), PDO_PAYLOAD_LEN)] = {0};
    bool operator==(const set_buffer_t& rhs) const { return memcmp(&header, &rhs.header, sizeof(set_buffer_t)) == 0; }
    bool operator!=(const set_buffer_t& rhs) const { return !(*this == rhs); }
    bool isValid()
    {
        if(header != MOTOR_STATE_HEADER && header != PAYLOAD_HEADER) return false;
        uint16_t crc_16 = 0;
        memcpy(&crc_16, crc16, 2);
        if(crc_16 == getCRC16(SN, sizeof(payload) + sizeof(SN))) return true;
        return false;
    }
    void prepare()
    {
        uint16_t crc_16 = getCRC16(SN, sizeof(payload) + sizeof(SN));
        memcpy(crc16, &crc_16, 2);
    }
    void setSN(uint32_t _SN) { memcpy(SN, &_SN, 4); }
    uint32_t getSN()
    {
        uint32_t ret = 0;
        memcpy(&ret, SN, 4);
        return ret;
    }
    [[nodiscard]] motor_set_t* set() const { return (motor_set_t*)payload; }
};

typedef void (*rx_payload_cb)(uint8_t *payload, uint8_t len);

class UARTProtocol
{
public:
    state_buffer_t state_buffer;
    set_buffer_t set_buffer;
    rx_payload_cb _payload_cb = nullptr;
    bool requestedTx = false;
    void PrepareStateBuf(motor_state_t *state);
    void PrepareSetBuf(motor_set_t *set);
    void PreparePayloadToStateBuf(char* payload, uint8_t len, uint32_t SN);
    void PreparePayloadToSetBuf(char* payload, uint8_t len);
    bool ReceiveStateBuf(char* buffer, uint16_t max_len);
    bool ReceiveSetBuf(char* buffer, uint16_t max_len, uint32_t SN);
    [[nodiscard]] bool IsAlive() const;
    uint32_t _SN = 0; // for master, will be updated in ReceiveStateBuf()
private:
    uint32_t last_tx_tick = 0;
    uint32_t last_rx_tick = 0;
};

bool UARTProtocol::ReceiveStateBuf(char* buffer, uint16_t max_len)
{
    uint16_t ret = 0;
    while(ret < max_len)
    {
        if(*(buffer + ret) == MOTOR_STATE_HEADER || *(buffer + ret) == PAYLOAD_HEADER) break;
        ret++;
    }
    // if(ret + sizeof(state_buffer) > max_len) return false;
    auto *rx_state = (state_buffer_t*)(buffer + ret);
    if((*rx_state).isValid())
    {
        memcpy(&state_buffer, buffer + ret, sizeof(state_buffer));
        if((*rx_state).header == PAYLOAD_HEADER)
        {
            // memcpy(&_SN, (*rx_state).payload, sizeof(_SN));
            if(_payload_cb) _payload_cb(state_buffer.payload + 4, sizeof(state_buffer.payload) - 4);
        }
        else _SN = (*rx_state).state()->SN;
        last_rx_tick = HAL_GetTick();
        return true;
    }
    return false;
}

bool UARTProtocol::ReceiveSetBuf(char* buffer, uint16_t max_len, uint32_t SN)
{
    if(max_len < sizeof(set_buffer)) return false;
    uint16_t ret = 0;
    auto *rx_set = (set_buffer_t*)(buffer + ret);
    while(ret < max_len)
    {
        rx_set = (set_buffer_t*)(buffer + ret);
        if(((*rx_set).header == MOTOR_STATE_HEADER || (*rx_set).header == PAYLOAD_HEADER) &&
           (*rx_set).getSN() == SN) break;
        ret++;
    }
    if(ret + sizeof(set_buffer) > max_len) return false;
    if((*rx_set).isValid())
    {
        memcpy(&set_buffer, rx_set, sizeof(set_buffer));
        if((*rx_set).header == PAYLOAD_HEADER)
        {
            if(_payload_cb) _payload_cb(set_buffer.payload, sizeof(set_buffer.payload));
        }
        last_rx_tick = HAL_GetTick();
        return true;
    }
    return false;
}

void UARTProtocol::PrepareStateBuf(motor_state_t *state)
{
    if(requestedTx) return;
    if(HAL_GetTick() - last_tx_tick < UART_KEEPALIVE_TICKS) // within keepalive period, we can choose to skip
    {
        if(*state == *(state_buffer.state())) return;
    }
    memcpy(state_buffer.payload, state, sizeof(motor_state_t));
    state_buffer.header = MOTOR_STATE_HEADER;
    state_buffer.prepare();
    last_tx_tick = HAL_GetTick();
    requestedTx = true;
}

void UARTProtocol::PrepareSetBuf(motor_set_t *set)
{
    if(requestedTx) return;
    if(_SN == 0) return;
    if(HAL_GetTick() - last_tx_tick < UART_KEEPALIVE_TICKS) // within keepalive period, we can choose to skip
    {
        if(*set == *(set_buffer.set())) return;
    }
    memcpy(set_buffer.payload, set, sizeof(motor_set_t));
    set_buffer.header = MOTOR_STATE_HEADER;
    set_buffer.setSN(_SN);
    set_buffer.prepare();
    last_tx_tick = HAL_GetTick();
    requestedTx = true;
}

void UARTProtocol::PreparePayloadToStateBuf(char* payload, uint8_t len, uint32_t SN) // payload uniqueness is guaranteed by upstream protocol
{
    // if(requestedTx) return;
    if(len > sizeof(state_buffer.payload) - 4) len = sizeof(state_buffer.payload) - 4;
    memset(state_buffer.payload, 0, sizeof(state_buffer.payload));
    memcpy(state_buffer.payload, &SN, sizeof(SN));
    memcpy(state_buffer.payload + 4, payload, len);
    state_buffer.header = PAYLOAD_HEADER;
    state_buffer.prepare();
    last_tx_tick = HAL_GetTick();
    requestedTx = true;
}

void UARTProtocol::PreparePayloadToSetBuf(char* payload, uint8_t len)
{
    // if(requestedTx) return;
    if(_SN == 0) return;
    if(len > sizeof(set_buffer.payload)) len = sizeof(set_buffer.payload);
    memset(set_buffer.payload, 0, sizeof(set_buffer.payload));
    memcpy(set_buffer.payload, payload, len);
    set_buffer.header = PAYLOAD_HEADER;
    set_buffer.setSN(_SN);
    set_buffer.prepare();
    last_tx_tick = HAL_GetTick();
    requestedTx = true;
}

bool UARTProtocol::IsAlive() const
{
    if(HAL_GetTick() - last_rx_tick < UART_TIMEOUT_TICKS) return true;
    return false;
}

#endif