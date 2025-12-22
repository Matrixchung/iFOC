#pragma once

/*
 * BoardConfig: singleton instance that can be accessed everywhere
 */

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

#include "Headers/Config/Board/board_config.h"
#include "config_nvm_wrapper.hpp"

namespace iFOC
{
namespace _const
{
    static constexpr uint8_t NVM_BOARD_CONFIG_STORE_SECTOR = 0;
}

DataType::ConfigNVMWrapper<DataType::Config::BoardConfig>& BoardConfig();

}

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif