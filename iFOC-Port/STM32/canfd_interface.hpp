// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#overview
// If an ODrive is configured with node_id = 0x3f, it does not send out any cyclic messages or responses
// , except for the special message Address.
#ifndef _STM32_CANFD_INTERFACE_HPP
#define _STM32_CANFD_INTERFACE_HPP

#include "base_protocol.hpp"
#include "can_opcodes_enum.h"
#include "global_include.h"
#include "string.h"

uint8_t GetCANID(uint32_t& id) { return (uint8_t)((id & 0x7E0) >> 5); };
uint8_t GetCANCommand(uint32_t& id) { return (uint8_t)(id & 0x1F); };
uint16_t GetCANFrameID(uint8_t node_id, uint16_t id) { return (uint16_t)((uint16_t)node_id << 5) | id; };

template<typename U>
class STM32CANFD
{
public:
    STM32CANFD(FDCAN_HandleTypeDef *_hcan, BaseProtocol<U>& _base) : hcan(_hcan), base(_base) {};
    void InitFilter();
    void InitHW();
    void Tick(float Ts);
    void IRQHandler(const FDCAN_RxHeaderTypeDef& header, uint8_t *buffer);
private:
    void ProcessDataFrame(uint32_t id, uint8_t *payload, uint8_t len);
    void ProcessRemoteFrame(uint32_t id);
    void ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index);
    void SendPayload(uint32_t id, uint8_t *payload, uint8_t len);
    FDCAN_HandleTypeDef *hcan;
    BaseProtocol<U>& base;
    uint8_t tx_buffer[8] = {0x00};
};

template<typename U>
void STM32CANFD<U>::ProcessDataFrame(uint32_t id, uint8_t *payload, uint8_t len)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCommand(id));
    // uint16_t node_id = GetCANID(id);
    float temp1 = 0.0f;
    uint16_t temp_u16 = 0;
    uint64_t temp_u64 = 0;
    switch(opcode)
    {
        case SET_INPUT_POS:
            memcpy(&temp1, payload, sizeof(float));
            base.SetEndpointValue(POS_INC_DEG, temp1);
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
                SendPayload(GetCANFrameID(base.node_id, RESPONSE_ENDPOINT), tx_buffer, 8);
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
                SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            }
            else if(len >= 7)
            {
                temp_u16 = payload[0];
                memcpy(&temp_u64, payload + 1, 6);
                if(temp_u64 == base.serial_number)
                {
                    base.node_id = (uint8_t)temp_u16;
                    InitFilter();
                }
                else if(temp_u16 == base.node_id)
                {
                    base.node_id = 0x3F;
                    InitFilter();
                }
            }
            break;
        case SET_LIMITS:
            // memcpy(&temp1, payload, 4);
            break;
        case SET_TRAJ_VEL_LIMIT:
            memcpy(&temp1, payload, 4);
            base.SetEndpointValue(POS_SPEED_LIMIT, temp1);
            break;
        case GET_IQ:
            temp1 = base.GetEndpointValue(IQ_OUT);
            memcpy(tx_buffer, &temp1, 4);
            temp1 = base.GetEndpointValue(CURRENT_IQ);
            memcpy(tx_buffer + 4, &temp1, 4);
            SendPayload(GetCANFrameID(base.node_id, GET_IQ), tx_buffer, 8);
            break;
        case GET_BUS_SENSE:
            temp1 = base.GetEndpointValue(VBUS);
            memcpy(tx_buffer, &temp1, 4);
            temp1 = base.GetEndpointValue(IBUS);
            memcpy(tx_buffer + 4, &temp1, 4);
            SendPayload(GetCANFrameID(base.node_id, GET_BUS_SENSE), tx_buffer, 8);
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

template<typename U>
void STM32CANFD<U>::ProcessRemoteFrame(uint32_t id)
{
    CAN_OPCODES opcode = static_cast<CAN_OPCODES>(GetCANCommand(id));
    switch(opcode)
    {
        case ADDRESS:
            tx_buffer[0] = base.node_id;
            memcpy(tx_buffer + 1, &base.serial_number, 6);
            SendPayload(GetCANFrameID(base.node_id, ADDRESS), tx_buffer, 7);
            break;
        default: break;
    }
}

template<typename U>
void STM32CANFD<U>::Tick(float Ts)
{

}

template<typename U>
void STM32CANFD<U>::IRQHandler(const FDCAN_RxHeaderTypeDef& header, uint8_t *buffer)
{
    // if(header.RTR == CAN_RTR_DATA)
    // {
    //     ProcessDataFrame(header.StdId, buffer, header.DLC);
    // }
    // else ProcessRemoteFrame(header.StdId);
    if(header.RxFrameType == FDCAN_DATA_FRAME)
    {
        ProcessDataFrame(header.Identifier, buffer, header.DataLength);
    }
    else ProcessRemoteFrame(header.Identifier);
}

template<typename U>
void STM32CANFD<U>::InitFilter()
{
    SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
    // HAL_CAN_Stop(hcan);
    if(base.node_id != 0x3F) ConfigFilter(base.node_id << 10, 0x7E0 << 5, 0, base.sub_dev_index * 2);
    // ConfigFilter(0x3F << 10, 0x7E0 << 5, 1, base.sub_dev_index * 2 + 1);
    if(base.sub_dev_index == 0) ConfigFilter(0x3F << 10, 0x7E0 << 5, 1, 1);
    // InitHW();
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
}


template<typename U>
void STM32CANFD<U>::InitHW()
{
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(hcan);
}

template<typename U>
void STM32CANFD<U>::ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = filter_index;
    sFilterConfig.FilterFIFOAssignment = fifo_index == 0 ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = id;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = mask;
    sFilterConfig.FilterMaskIdLow = 0xFFFC;
    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        while(1);
    }
}

template<typename U>
void STM32CANFD<U>::SendPayload(uint32_t id, uint8_t *payload, uint8_t len)
{
    uint32_t mailbox = 0;
    CAN_TxHeaderTypeDef header;
    header.TransmitGlobalTime = DISABLE;
    header.IDE = CAN_ID_STD;
    header.StdId = id;
    header.RTR = CAN_RTR_DATA;
    if(len > 8) len = 8;
    header.DLC = len;
    memcpy(tx_buffer, payload, len);
    HAL_CAN_AddTxMessage(hcan, &header, tx_buffer, &mailbox);
}

#endif