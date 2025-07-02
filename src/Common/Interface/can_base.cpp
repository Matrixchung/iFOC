#include "can_base.hpp"

namespace iFOC::HAL
{
CANBase::CANBase()
{
    tx_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(tx_sem);
}

CANBase::~CANBase()
{
    vSemaphoreDelete(tx_sem);
}

void CANBase::ProcessIncomingMsg(const DataType::Comm::CANMessage& msg) const
{
    for(const auto& cb : cb_list) cb(msg);
}

void CANBase::RegisterRxHandler(EventCallback cb)
{
    cb_list.push_back(cb);
}

}