#pragma once

#include "../../Common/Interface/canfd_base.hpp"
#include "hal_const.h"

#if defined(AT32WK_ENV) && defined(CAN_MODULE_ENABLED) && defined(IFOC_CANFD_AVAILABLE)

namespace iFOC::HAL
{
class CANFD final : public CANFDBase
{
public:
    explicit CANFD(can_type* _hcan);
    FuncRetCode Init(DataType::Comm::CANBaudrate arbit_baud, DataType::Comm::CANBaudrate data_baud) override;
    FuncRetCode TransmitMessage(DataType::Comm::CANFDMessage& msg) override;
    FuncRetCode SetHWFilter(uint8_t filter_idx, uint32_t id_u32, uint32_t mask_u32, bool ext_only) override;
    void OnIRQ() const;
    void OnErrorIRQ() const;
private:
    can_type* hcan;
};
}

#endif