#ifndef _FOC_CAN_PROTOCOL_HPP
#define _FOC_CAN_PROTOCOL_HPP

#include "base_protocol.hpp"
#include "can_base.h"
#include "can_opcodes_enum.h"

uint8_t GetCANNodeID(uint32_t id) { return (uint8_t)((id & 0x7E0) >> 5); };
uint8_t GetCANCmdID(uint32_t id) { return (uint8_t)(id & 0x1F); };
uint16_t GetCANFrameID(uint8_t node_id, uint16_t id) { return (uint16_t)((uint16_t)node_id << 5) | id; };

template<typename Intf, typename U>
class CANProtocol
{
public:
    CANProtocol(Intf& _intf, BaseProtocol<U>& _base) : interface(_intf), base(_base) 
    {
        static_assert(std::is_base_of<CANBase, Intf>::value, "CAN Implementation must be derived from CANBase");
    };
    void Init(bool initHW = false);
    void Tick(float Ts);
    void OnDataFrame(uint32_t id, uint8_t *payload, uint8_t len);
    void OnRemoteFrame(uint32_t id);
    uint8_t GetNodeID() { return base.node_id; };
private:
    Intf& interface;
    BaseProtocol<U>& base;
    uint8_t tx_buffer[8] = {0x00};
};

template<typename Intf, typename U>
void CANProtocol<Intf, U>::Init(bool initHW)
{
    if(base.node_id != 0x3F) interface.ConfigFilter(base.node_id, 0x7E0, 0, base.sub_dev_index * 2);
    if(base.sub_dev_index == 0) interface.ConfigFilter(0x3F, 0x7E0, 1, 1); // Config a global broadcast filter in FIFO1
    if(initHW) interface.InitHW();
}

template<typename Intf, typename U>
void CANProtocol<Intf, U>::Tick(float Ts)
{
    if(base.pos_target_sent && base.GetEndpointValue(TRAJ_POS_STATE))
    {
        base.pos_target_sent = false;
        memset(tx_buffer, 0, 8);
        uint16_t temp_u16 = 0;
        temp_u16 = (uint16_t)base.GetEndpointValue(DRIVE_ERROR_CODE);
        tx_buffer[0] = (uint8_t)temp_u16;
        tx_buffer[1] = (uint8_t)(temp_u16 << 8);
        tx_buffer[4] = (uint8_t)base.GetEndpointValue(OUTPUT_STATE);
        tx_buffer[6] = (uint8_t)base.GetEndpointValue(TRAJ_POS_STATE);
        interface.SendPayload(GetCANFrameID(base.node_id, HEARTBEAT), tx_buffer, 8);
    }
}

template<typename Intf, typename U>
void CANProtocol<Intf, U>::OnDataFrame(uint32_t id, uint8_t *payload, uint8_t len)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCmdID(id));
    float temp1 = 0.0f;
    uint16_t temp_u16 = 0;
    uint64_t temp_u64 = 0;
    int32_t temp_i32 = 0;
    switch(opcode)
    {
        case HEARTBEAT:
            memset(tx_buffer, 0, 8);
            temp_u16 = (uint16_t)base.GetEndpointValue(DRIVE_ERROR_CODE);
            tx_buffer[0] = (uint8_t)temp_u16;
            tx_buffer[1] = (uint8_t)(temp_u16 << 8);
            tx_buffer[4] = (uint8_t)base.GetEndpointValue(OUTPUT_STATE);
            tx_buffer[6] = (uint8_t)base.GetEndpointValue(TRAJ_POS_STATE);
            interface.SendPayload(GetCANFrameID(base.node_id, HEARTBEAT), tx_buffer, 8);
            break;
        case ESTOP:
            base.SetEndpointValue(OUTPUT_STATE, 0.0f);
            break;
        case GET_ERROR:
            memset(tx_buffer, 0, 8);
            temp_u16 = (uint16_t)base.GetEndpointValue(DRIVE_ERROR_CODE);
            tx_buffer[0] = (uint8_t)temp_u16;
            tx_buffer[1] = (uint8_t)(temp_u16 << 8);
            interface.SendPayload(GetCANFrameID(base.node_id, GET_ERROR), tx_buffer, 8);
            break;
        case SET_INPUT_POS:
            temp_i32 = (int32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
            base.SetEndpointValue(POS_INC_DEG, (float)temp_i32);
            break;
        case ACCESS_ENDPOINT:
            if(len < 4) break;
            if(payload[0] == 0) // READ endpoint
            {
                memset(tx_buffer, 0, 8);
                temp_u16 = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
                temp1 = base.GetEndpointValue(GetEndpointFromIndex(temp_u16));
                memcpy(tx_buffer + 4, &temp1, 4);
                tx_buffer[1] = payload[1];
                tx_buffer[2] = payload[2];
                interface.SendPayload(GetCANFrameID(base.node_id, RESPONSE_ENDPOINT), tx_buffer, 8);
            }
            else if(len == 8) // WRITE endpoint
            {

            }
            break;
        case ADDRESS:
            if(len == 0)
            {
                tx_buffer[0] = base.node_id;
                memcpy(tx_buffer + 1, &base.serial_number, 6);
                interface.SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            }
            else if(len >= 7)
            {
                temp_u16 = payload[0];
                memcpy(&temp_u64, payload + 1, 6);
                if(temp_u64 == base.serial_number)
                {
                    base.node_id = (uint8_t)temp_u16;
                    Init();
                }
                else if(temp_u16 == base.node_id)
                {
                    base.node_id = 0x3F;
                    Init();
                }
            }
            break;
        case SET_STATE:
            if(payload[0] == 0) base.SetEndpointValue(OUTPUT_STATE, 0.0f);
            else base.SetEndpointValue(OUTPUT_STATE, 1.0f);
            break;
        case GET_ESTIMATES:
            memset(tx_buffer, 0, 8);
            temp1 = base.GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD);
            memcpy(tx_buffer, &temp1, 4);
            temp1 = base.GetEndpointValue(ESTIMATED_SPEED);
            memcpy(tx_buffer + 4, &temp1, 4);
            interface.SendPayload(GetCANFrameID(base.node_id, GET_ESTIMATES), tx_buffer, 8);
            break;
        case SET_LIMITS:
            break;
        case SET_TRAJ_VEL_LIMIT:
            memcpy(&temp1, payload, 4);
            base.SetEndpointValue(POS_SPEED_LIMIT, temp1);
            break;
        case GET_IQ:
            temp1 = base.GetEndpointValue(IQ_SET);
            memcpy(tx_buffer, &temp1, 4);
            temp1 = base.GetEndpointValue(CURRENT_IQ);
            memcpy(tx_buffer + 4, &temp1, 4);
            interface.SendPayload(GetCANFrameID(base.node_id, GET_IQ), tx_buffer, 8);
            break;
        case GET_BUS_SENSE:
            temp1 = base.GetEndpointValue(VBUS);
            memcpy(tx_buffer, &temp1, 4);
            temp1 = base.GetEndpointValue(IBUS);
            memcpy(tx_buffer + 4, &temp1, 4);
            interface.SendPayload(GetCANFrameID(base.node_id, GET_BUS_SENSE), tx_buffer, 8);
            break;
        case REBOOT:
            if(len != 1) break;
            if(payload[0] == 0) NVIC_SystemReset();
            else if(payload[0] == 1) ; // Save config
            else if(payload[0] == 2) ; // Erase config
            break;
        default: break;
    }
}

template<typename Intf, typename U>
void CANProtocol<Intf, U>::OnRemoteFrame(uint32_t id)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCmdID(id));
    switch(opcode)
    {
        case ADDRESS:
            tx_buffer[0] = base.node_id;
            memcpy(tx_buffer + 1, &base.serial_number, 6);
            interface.SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            break;
        default: break;
    }
}

#endif