#pragma once

#include <cstdint>

#include "hal_impl.hpp"
#include "func_ret_code.h"

namespace iFOC::HAL::NVM
{
static FuncRetCode Write(uint8_t sector, const uint8_t *buffer, uint16_t size)
{
    auto ret = EraseSector(sector);
    if(ret != FuncRetCode::OK) return ret;
    return Write_NoErase(sector, buffer, size);
}

static void WritePVD(uint8_t sector, const uint8_t *buffer, uint16_t size)
{
    Write_NoErase(sector, buffer, size);
}
}