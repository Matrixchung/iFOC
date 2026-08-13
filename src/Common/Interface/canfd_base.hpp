#pragma once

#include <cstdint>
#include <functional>
#include "../foc_types.hpp"
#include "../../DataType/Headers/Comm/can_baudrate.h"
#include "../../DataType/Headers/Comm/can_message.h"

namespace iFOC::HAL
{
class CANFDBase
{
    DELETE_COPY_CONSTRUCTOR(CANFDBase);
    OVERRIDE_NEW();
public:
    // if false: next registered event can continue process. true means message has been successfully handled by current callback.
    using EventCallback = std::function<bool(const DataType::Comm::CANFDMessage&)>;
protected:
    Vector<EventCallback> cb_list{};
public:
    CANFDBase() = default;
    virtual ~CANFDBase() = default;

    // For custom usage, we are allowed to extend arbitration phase baudrate to over 1Mbps
    virtual FuncRetCode Init(DataType::Comm::CANBaudrate arbit_baud, DataType::Comm::CANBaudrate data_baud) = 0;

    virtual FuncRetCode TransmitMessage(DataType::Comm::CANFDMessage& msg) = 0;

    virtual FuncRetCode SetHWFilter(uint8_t filter_idx,
                                    uint32_t id_u32,
                                    uint32_t mask_u32,
                                    bool ext_only) { return FuncRetCode::NOT_SUPPORTED; };

    void ProcessIncomingMsg(const DataType::Comm::CANFDMessage& msg) const;

    void RegisterRxHandler(const EventCallback& cb);

    static DataType::Comm::canfd_dlc GetDLCFromLength(uint8_t data_length);
};
}