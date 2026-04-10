#pragma once

#include "../../Common/Interface/can_base.hpp"
#include "../hal_const.h"

#if defined(GD32_ENV)

namespace iFOC::HAL
{

class CAN final : public CANBase
{
public:
    explicit CAN(uint32_t _hcan);
    FuncRetCode Init(DataType::Comm::CANBaudrate baud) override;
    FuncRetCode TransmitMessage(DataType::Comm::CANMessage& msg) override;
    FuncRetCode SetHWFilter(uint8_t filter_idx, uint32_t id_u32, uint32_t mask_u32, bool ext_only, bool accept_rtr) override;
    void OnIRQ() const;
private:
    bool IsMailboxEmpty(uint8_t mb) const;
    uint32_t hcan;
};

}

#endif