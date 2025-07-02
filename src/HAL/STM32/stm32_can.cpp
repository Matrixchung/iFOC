#include "stm32_can.hpp"

#if defined(USE_STM32_FDCAN) || defined(USE_STM32_CAN)

namespace iFOC::HAL
{
STM32CAN::STM32CAN(can_handle_t* _hcan) : CANBase(), hcan(_hcan) {}

FuncRetCode STM32CAN::Init(DataType::Comm::CANBaudrate baud)
{
#ifdef USE_STM32_FDCAN
    if(HAL_FDCAN_DeInit(hcan) != HAL_OK) return FuncRetCode::PARAM_NOT_EXIST;
#ifdef STM32G431xx
    switch(baud)
    {
        case DataType::Comm::CANBaudrate::BAUD_1_MBPS:
        {
            hcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
            hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
            hcan->Init.Mode = FDCAN_MODE_NORMAL;
            hcan->Init.AutoRetransmission = ENABLE;
            hcan->Init.TransmitPause = DISABLE;
            hcan->Init.ProtocolException = DISABLE;
            hcan->Init.NominalPrescaler = 5;
            hcan->Init.NominalSyncJumpWidth = 9;
            hcan->Init.NominalTimeSeg1 = 24;
            hcan->Init.NominalTimeSeg2 = 9;
            hcan->Init.DataPrescaler = 5;
            hcan->Init.DataSyncJumpWidth = 9;
            hcan->Init.DataTimeSeg1 = 24;
            hcan->Init.DataTimeSeg2 = 9;
            hcan->Init.StdFiltersNbr = 1;
            hcan->Init.ExtFiltersNbr = 0;
            hcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
            break;
        }
        default: break;
    }
#endif
    if(HAL_FDCAN_Init(hcan) != HAL_OK) return FuncRetCode::HARDWARE_ERROR;
    FDCAN_FilterTypeDef sFilterConfig
    {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0x00000000,
        .FilterID2 = 0x00000000
    };
    HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigTxDelayCompensation(hcan, hcan->Init.DataPrescaler * hcan->Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(hcan);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    if(HAL_FDCAN_Start(hcan) == HAL_OK) return FuncRetCode::OK;
    return FuncRetCode::HARDWARE_ERROR;
#else
    return FuncRetCode::OK; // TODO
#endif
}

FuncRetCode STM32CAN::TransmitMessage(DataType::Comm::CANMessage& msg)
{
    if(xSemaphoreTakeAuto(tx_sem, WRITE_TIMEOUT_MS) == pdTRUE)
    {
#ifdef USE_STM32_FDCAN
        FDCAN_TxHeaderTypeDef header{};
        header.Identifier = msg.cob_id;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = msg.is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
        header.DataLength = msg.len;
        // memcpy(tx_buffer, msg.data, msg.len);
        if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, msg.data) != HAL_OK)
        {
            xSemaphoreGiveAuto(tx_sem);
            return FuncRetCode::BUFFER_FULL;
        }
#else
#endif
        xSemaphoreGiveAuto(tx_sem);
        return FuncRetCode::OK;
    }
    return FuncRetCode::REMOTE_TIMEOUT;
}

void STM32CAN::OnIRQ(can_handle_t *_can)
{
    if(_can != hcan) return;
#ifdef USE_STM32_FDCAN
    FDCAN_RxHeaderTypeDef header{};
    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &header, rx_buffer);
    DataType::Comm::CANMessage rx_msg
    {
        .cob_id = (uint16_t)header.Identifier,
        .is_rtr = header.RxFrameType == FDCAN_REMOTE_FRAME,
        .len = (uint8_t)_constrain(header.DataLength, 0, sizeof(DataType::Comm::CANMessage::data))
    };
    memcpy(rx_msg.data, rx_buffer, rx_msg.len);
    ProcessIncomingMsg(rx_msg);
#else
#endif
}
}

#endif