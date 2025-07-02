#pragma once

#include <cstdint>
#include <functional>
#include "foc_types.hpp"
#include "../../DataType/Headers/Comm/can_baudrate.h"
#include "../../DataType/Headers/Comm/can_message.h"

namespace iFOC::HAL
{
class CANBase
{
    DELETE_COPY_CONSTRUCTOR(CANBase);
    OVERRIDE_NEW();
public:
    using EventCallback = std::function<void(const DataType::Comm::CANMessage&)>;
protected:
    static constexpr TickType_t WRITE_TIMEOUT_MS = 100;
    Vector<EventCallback> cb_list{};
    SemaphoreHandle_t tx_sem = nullptr;
public:
    CANBase();
    virtual ~CANBase();

    virtual FuncRetCode Init(DataType::Comm::CANBaudrate baud) = 0;

    virtual FuncRetCode TransmitMessage(DataType::Comm::CANMessage& msg) = 0;

    void ProcessIncomingMsg(const DataType::Comm::CANMessage& msg) const;

    void RegisterRxHandler(EventCallback cb);
};
}