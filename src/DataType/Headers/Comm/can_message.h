#pragma once

namespace iFOC::DataType::Comm
{
struct CANMessage
{
    uint16_t cob_id = 0;
    bool is_rtr = false;
    uint8_t len = 0;
    uint8_t data[8]{};
};
}