#pragma once

#include <cstdint>

namespace iFOC::DataType::Base
{
enum class FuncRetCode : uint8_t
{
    OK = 0,
    INVALID_INPUT = 1,
    INVALID_RESULT = 2,
    BUFFER_FULL = 3,
    ACCESS_VIOLATION = 4,
    CRC_MISMATCH = 5,
    PARAM_OUT_BOUND = 6,
    HARDWARE_ERROR = 7,
    REMOTE_TIMEOUT = 8,
    NOT_SUPPORTED = 9,
    PARAM_NOT_EXIST = 10,
    PARAM_DUPLICATED = 11,
    BUSY = 12,
};
}