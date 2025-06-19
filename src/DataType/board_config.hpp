#pragma once

/*
 * BoardConfig: singleton instance that can be accessed everywhere
 */

#include "Headers/Config/Board/board_config.h"
#include "config_nvm_wrapper.hpp"

namespace iFOC
{
namespace _const
{
    static constexpr uint8_t NVM_BOARD_CONFIG_STORE_SECTOR = 0;
}

// static auto& BoardConfig()
// {
//     static DataType::ConfigNVMWrapper<DataType::Config::BoardConfig> instance(DataType::Base::ProtoHeader::BOARD_CONFIG,
//                                                                               _const::NVM_BOARD_CONFIG_STORE_SECTOR);
//     return instance;
// }

extern DataType::ConfigNVMWrapper<DataType::Config::BoardConfig> BoardConfig;

}