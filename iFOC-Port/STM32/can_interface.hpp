#ifndef _STM32_CAN_INTERFACE_HPP
#define _STM32_CAN_INTERFACE_HPP

#include "can_protocol.hpp"
#include "global_include.h"

class STM32CAN : public CANBase
{
public:
    STM32CAN(CAN_HandleTypeDef *_hcan) : hcan(_hcan) {};
    void ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index) override;
    void InitHW() override;
    void SendPayload(uint32_t id, uint8_t *payload, uint8_t len) override;
    template<class ... instances>
    void OnIRQ(uint32_t fifo_index, instances&... args);
private:
    CAN_HandleTypeDef *hcan;
    template<class T>
    void _irq(T instance, uint8_t &id, CAN_RxHeaderTypeDef& rx_header);
};

void STM32CAN::ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index)
{
    SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = filter_index;
    sFilterConfig.FilterFIFOAssignment = fifo_index == 0 ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = id << 10;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = mask << 5;
    sFilterConfig.FilterMaskIdLow = 0xFFFC;
    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) Error_Handler();
    CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
}

void STM32CAN::InitHW()
{
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(hcan);
}

void STM32CAN::SendPayload(uint32_t id, uint8_t *payload, uint8_t len)
{
    __unused uint32_t mailbox = 0;
    CAN_TxHeaderTypeDef header;
    header.TransmitGlobalTime = DISABLE;
    header.IDE = CAN_ID_STD;
    header.StdId = id;
    header.RTR = CAN_RTR_DATA;
    if(len > 8) len = 8;
    header.DLC = len;
    HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
}

template<class ... instances>
void STM32CAN::OnIRQ(uint32_t fifo_index, instances&... args)
{
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, fifo_index, &rx_header, rx_buffer);
    uint8_t id = GetCANNodeID(rx_header.StdId);
    (_irq(args, id, rx_header), ...);
}

template<class T>
void STM32CAN::_irq(T instance, uint8_t& id, CAN_RxHeaderTypeDef& rx_header)
{
    if(id == instance.GetNodeID() || id == 0x3F)
    {
        if(rx_header.RTR == CAN_RTR_DATA) instance.OnDataFrame(rx_header.StdId, rx_buffer, rx_header.DLC);
        else instance.OnRemoteFrame(rx_header.StdId);
    }
}

#endif