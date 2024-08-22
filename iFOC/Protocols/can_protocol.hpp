#ifndef _FOC_CAN_PROTOCOL_HPP
#define _FOC_CAN_PROTOCOL_HPP

#include "base_protocol.hpp"
#include "can_base.h"
#include "can_opcodes_enum.h"
#include "platform.hpp"

// Identifier will flash for (x) times in BLINK_TIME, x = sub_dev_index + 1

#define CAN_IDENTIFIER_BLINK_TIME 1.0f
#define CAN_IDENTIFIER_FLASH_TIME 0.1f
#define CAN_HEARTBEAT_INTERVAL 0.1f
#define CAN_GET_ESTIMATES_INTERVAL 0.01f

uint8_t GetCANNodeID(uint32_t id) { return (uint8_t)((id & 0x7E0) >> 5); };
uint8_t GetCANCmdID(uint32_t id) { return (uint8_t)(id & 0x1F); };
uint16_t GetCANFrameID(uint8_t node_id, uint16_t id) { return (uint16_t)((uint16_t)node_id << 5) | id; };

uint8_t is_identifying = 0;
uint8_t delay_send_addr_flag = 0;

template<class Intf, class U>
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
    void SendPacket(CAN_OPCODES opcode);
    uint8_t GetNodeID() { return base.node_id; };
private:
    Intf& interface;
    BaseProtocol<U>& base;
    uint8_t tx_buffer[8] = {0x00};
    float identify_blink_timer = 0.0f;
    float heartbeat_timer = 0.0f;
    float estimates_timer = 0.0f;
    uint8_t flash_timer = 0;
};

template<class Intf, class U>
void CANProtocol<Intf, U>::Init(bool initHW)
{
    if(base.node_id != 0x3F) interface.ConfigFilter(base.node_id, 0x7E0, 0, base.sub_dev_index * 2);
    if(base.sub_dev_index == 0) interface.ConfigFilter(0x3F, 0x7E0, 1, 1); // Config a global broadcast filter in FIFO1
    if(initHW) interface.InitHW();
}

template<class Intf, class U>
void CANProtocol<Intf, U>::Tick(float Ts)
{
    if(base.node_id != 0x3F)
    {
        if(is_identifying == base.sub_dev_index + 1)
        {
            identify_blink_timer += Ts;
            if(identify_blink_timer >= CAN_IDENTIFIER_BLINK_TIME - 2.0f * ((float)base.sub_dev_index * CAN_IDENTIFIER_FLASH_TIME))
            {
                base.SetEndpointValue(INDICATOR_TOGGLE, 0.0f);
                identify_blink_timer -= CAN_IDENTIFIER_FLASH_TIME;
                flash_timer++;
                if(flash_timer > (base.sub_dev_index * 2) + 1)
                {
                    identify_blink_timer = 0.0f;
                    flash_timer = 0;
                }
            }
        }
        else 
        {
            identify_blink_timer = 0.0f;
            flash_timer = 0;
        }
        estimates_timer += Ts;
        heartbeat_timer += Ts;
        if(estimates_timer >= CAN_GET_ESTIMATES_INTERVAL)
        {
            SendPacket(GET_ESTIMATES);
            estimates_timer = 0.0f;
        }
        else if(heartbeat_timer >= CAN_HEARTBEAT_INTERVAL)
        {
            SendPacket(HEARTBEAT);
            heartbeat_timer = 0.0f;
        }
    }
    if(delay_send_addr_flag & (1 << base.sub_dev_index))
    {
        SendPacket(ADDRESS);
        delay_send_addr_flag &= ~(1 << base.sub_dev_index);
    }

}

template<class Intf, class U>
void CANProtocol<Intf, U>::SendPacket(CAN_OPCODES opcode)
{
    switch(opcode)
    {
        case HEARTBEAT:
            {
                memset(tx_buffer, 0, 8);
                uint16_t temp_u16 = (uint16_t)base.GetEndpointValue(DRIVE_ERROR_CODE);
                tx_buffer[0] = (uint8_t)temp_u16;
                tx_buffer[1] = (uint8_t)(temp_u16 << 8);
                tx_buffer[4] = (uint8_t)base.GetEndpointValue(OUTPUT_STATE);
                tx_buffer[6] = (uint8_t)base.GetEndpointValue(TRAJ_POS_STATE);
                interface.SendPayload(GetCANFrameID(base.node_id, HEARTBEAT), tx_buffer, 8);
            }
            break;
        case GET_ERROR:
            {
                memset(tx_buffer, 0, 8);
                uint16_t temp_u16 = (uint16_t)base.GetEndpointValue(DRIVE_ERROR_CODE);
                tx_buffer[0] = (uint8_t)temp_u16;
                tx_buffer[1] = (uint8_t)(temp_u16 << 8);
                interface.SendPayload(GetCANFrameID(base.node_id, GET_ERROR), tx_buffer, 8);
            }
            break;
        case ADDRESS:
            tx_buffer[0] = base.node_id;
            memcpy(tx_buffer + 1, &base.serial_number, 6);
            interface.SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            break;
        case GET_ESTIMATES:
            {
                memset(tx_buffer, 0, 8);
                float temp1 = base.GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD);
                memcpy(tx_buffer, &temp1, 4);
                temp1 = base.GetEndpointValue(ESTIMATED_SPEED);
                memcpy(tx_buffer + 4, &temp1, 4);
                interface.SendPayload(GetCANFrameID(base.node_id, GET_ESTIMATES), tx_buffer, 8);
            }
            break;
        default: break;
    }
}

template<class Intf, class U>
void CANProtocol<Intf, U>::OnDataFrame(uint32_t id, uint8_t *payload, uint8_t len)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCmdID(id));
    switch(opcode)
    {
        case HEARTBEAT:
            SendPacket(HEARTBEAT);
            break;
        case ESTOP:
            base.SetEndpointValue(OUTPUT_STATE, 0.0f);
            break;
        case GET_ERROR:
            SendPacket(GET_ERROR);
            break;
        case SET_INPUT_POS:
            {
                int32_t temp_i32 = (int32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
                base.SetEndpointValue(POS_INC_DEG, (float)temp_i32);
            }
            break;
        case ACCESS_ENDPOINT:
            if(len < 4) break;
            if(payload[0] == 0) // READ endpoint
            {
                memset(tx_buffer, 0, 8);
                uint16_t temp_u16 = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
                float temp1 = base.GetEndpointValue(GetEndpointFromIndex(temp_u16));
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
                SendPacket(ADDRESS);
            }
            else if(len >= 7)
            {
                uint8_t temp_u8 = payload[0]; // Node_ID
                uint64_t temp_u64 = 0; // Serial_Number
                memcpy(&temp_u64, payload + 1, 6);
                uint8_t caller_id = GetCANNodeID(id);
                if(caller_id == 0x3F)
                {
                    if(temp_u64 == base.serial_number)
                    {
                        base.node_id = temp_u8;
                        Init();
                    }
                }
                else
                {
                    if(temp_u64 == base.serial_number || base.serial_number == 0)
                    {
                        base.node_id = temp_u8;
                        Init();
                    }
                }
                // if(temp_u64 == base.serial_number)
                // {
                //     base.node_id = (uint8_t)temp_u16;
                //     Init();
                // }
                // else if(temp_u16 == base.node_id)
                // {
                //     base.node_id = 0x3F;
                //     Init();
                // }
            }
            break;
        case SET_STATE:
            if(payload[0] == 0) base.SetEndpointValue(OUTPUT_STATE, 0.0f);
            else base.SetEndpointValue(OUTPUT_STATE, 1.0f);
            break;
        case GET_ESTIMATES:
            SendPacket(GET_ESTIMATES);
            break;
        case SET_LIMITS:
            break;
        case SET_TRAJ_VEL_LIMIT:
            {
                float temp1 = 0.0f;
                memcpy(&temp1, payload, 4);
                base.SetEndpointValue(POS_SPEED_LIMIT, temp1);
            }
            break;
        case GET_IQ:
            {
                float temp1 = base.GetEndpointValue(IQ_SET);
                memcpy(tx_buffer, &temp1, 4);
                temp1 = base.GetEndpointValue(CURRENT_IQ);
                memcpy(tx_buffer + 4, &temp1, 4);
                interface.SendPayload(GetCANFrameID(base.node_id, GET_IQ), tx_buffer, 8);
            }
            break;
        case GET_BUS_SENSE:
            {
                float temp1 = base.GetEndpointValue(VBUS);
                memcpy(tx_buffer, &temp1, 4);
                temp1 = base.GetEndpointValue(IBUS);
                memcpy(tx_buffer + 4, &temp1, 4);
                interface.SendPayload(GetCANFrameID(base.node_id, GET_BUS_SENSE), tx_buffer, 8);
            }
            break;
        case IDENTIFY:
            {
                if(payload[0] == 0)
                {
                    is_identifying = 0;
                    base.SetEndpointValue(INDICATOR_STATE, 0.0f);
                }
                else if(base.node_id != 0x3F)
                {
                    is_identifying = base.sub_dev_index + 1;
                }
            }
            break;
        case REBOOT:
            {
                if(len != 1) break;
                if(payload[0] == 0) NVIC_SystemReset();
                else if(payload[0] == 1) 
                {
                    base.SetEndpointValue(CONFIGURATION, 1.0f); // Save config
                }
                else if(payload[0] == 2) 
                {
                    base.SetEndpointValue(CONFIGURATION, 2.0f); // Erase config
                }
            }
            break;
        case BOOTLOADER:
            JumpToBootloader();
            break;
        default: break;
    }
}

template<class Intf, class U>
void CANProtocol<Intf, U>::OnRemoteFrame(uint32_t id)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCmdID(id));
    switch(opcode)
    {
        case ADDRESS:
            delay_send_addr_flag |= (1 << base.sub_dev_index);
            // tx_buffer[0] = base.node_id;
            // memcpy(tx_buffer + 1, &base.serial_number, 6);
            // interface.SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            break;
        default: break;
    }
}

#endif