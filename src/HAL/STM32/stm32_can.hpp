#pragma once

#include "can_base.hpp"
#include "main.h"

#if defined(HAL_FDCAN_MODULE_ENABLED)
#define USE_STM32_FDCAN
#else defined(HAL_CAN_MODULE_ENABLED)
#define USE_STM32_CAN
#endif

#if defined(USE_STM32_FDCAN) || defined(USE_STM32_CAN)

namespace iFOC::HAL
{
class STM32CAN final : public CANBase
{
private:
#ifdef USE_STM32_FDCAN
    using can_handle_t = FDCAN_HandleTypeDef;
#else
    using can_handle_t = CAN_HandleTypeDef;
#endif
public:
    explicit STM32CAN(can_handle_t *_hcan);
    FuncRetCode Init(DataType::Comm::CANBaudrate baud) override;
    FuncRetCode TransmitMessage(DataType::Comm::CANMessage& msg) override;
    void OnIRQ(can_handle_t *_can);
private:
    can_handle_t *hcan;
    uint8_t rx_buffer[8]{};
    // uint8_t tx_buffer[16]{};
};
}

#endif