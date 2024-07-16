#ifndef _STM32_CANFD_INTERFACE_HPP
#define _STM32_CANFD_INTERFACE_HPP

#include "can_protocol.hpp"
#include "global_include.h"

class STM32CANFD : public CANBase
{
public:
    STM32CANFD(FDCAN_HandleTypeDef *_hcan) : hcan(_hcan) {};
    void ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index) override;
    void InitHW() override;
    void SendPayload(uint32_t id, uint8_t *payload, uint8_t len) override;
    template<class ... instances>
    void OnIRQ(uint32_t fifo_index, instances&... args);
private:
    FDCAN_HandleTypeDef *hcan;
    template<class T>
    void _irq(T instance, uint8_t &id, FDCAN_RxHeaderTypeDef& rx_header);
};

void STM32CANFD::ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = filter_index;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = fifo_index == 0 ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = id << 5;
    sFilterConfig.FilterID2 = mask;
    sFilterConfig.RxBufferIndex = 0;
    if(HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) Error_Handler();
}

void STM32CANFD::InitHW()
{
    HAL_FDCAN_ConfigTxDelayCompensation(hcan, hcan->Init.DataPrescaler * hcan->Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(hcan);
    HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(hcan);
}

void STM32CANFD::SendPayload(uint32_t id, uint8_t *payload, uint8_t len)
{
    FDCAN_TxHeaderTypeDef header;
    header.IdType = FDCAN_STANDARD_ID;
    header.Identifier = id;
    header.TxFrameType = FDCAN_DATA_FRAME;
    if(len > 8) len = 8;
    header.DataLength = len;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, payload);
}

template<class ... instances>
void STM32CANFD::OnIRQ(uint32_t fifo_index, instances&... args)
{
    FDCAN_RxHeaderTypeDef rx_header;
    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_buffer);
    uint8_t id = GetCANNodeID(rx_header.Identifier);
    (_irq(args, id, rx_header), ...);
}

template<class T>
void STM32CANFD::_irq(T instance, uint8_t& id, FDCAN_RxHeaderTypeDef& rx_header)
{
    if(id == instance.GetNodeID() || id == 0x3F)
    {
        if(rx_header.RxFrameType == FDCAN_DATA_FRAME) instance.OnDataFrame(rx_header.Identifier, rx_buffer, rx_header.DataLength);
        else instance.OnRemoteFrame(rx_header.Identifier);
    }
}

#endif