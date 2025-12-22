#pragma once

#include "../../Common/Interface/can_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(CAN_MODULE_ENABLED)

namespace iFOC::HAL
{
class CAN final : public CANBase
{
public:
    explicit CAN(can_type *_hcan);
    FuncRetCode Init(DataType::Comm::CANBaudrate baud) override;
    FuncRetCode TransmitMessage(DataType::Comm::CANMessage& msg) override;
    FuncRetCode SetHWFilter(uint8_t filter_idx, uint32_t id_u32, uint32_t mask_u32) override;
    void OnIRQ() const;
private:
    can_type* hcan;
};
}

#endif