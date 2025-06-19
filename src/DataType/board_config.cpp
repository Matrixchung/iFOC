#include "board_config.hpp"

namespace iFOC
{
DataType::ConfigNVMWrapper<DataType::Config::BoardConfig> BoardConfig(DataType::Base::ProtoHeader::BOARD_CONFIG,
                                                                      _const::NVM_BOARD_CONFIG_STORE_SECTOR);
}