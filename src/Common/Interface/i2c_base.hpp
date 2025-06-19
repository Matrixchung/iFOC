#pragma once

#include <cstdint>
#include <type_traits>
#include <cstddef>
#include "func_ret_code.h"
#include "foc_types.hpp"

namespace iFOC::HAL
{
class I2CBase
{
    DELETE_COPY_CONSTRUCTOR(I2CBase);
    OVERRIDE_NEW();
public:
    I2CBase() = default;
    virtual FuncRetCode Init() { return FuncRetCode::OK; };
    virtual FuncRetCode WriteBytes(uint8_t addr, const uint8_t* data, uint16_t size) = 0;
    virtual FuncRetCode ReadBytes(uint8_t addr, uint8_t* data, uint16_t size) = 0;
};

template<typename T>
concept I2CImpl = std::is_base_of<I2CBase, T>::value;
}