/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/misconfigured_area.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_MISCONFIGURED_AREA_H
#define BASE_MISCONFIGURED_AREA_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class MisconfiguredArea : uint32_t
{
  NONE = 0,
  MOTOR_INIT_COMPONENTS_MISSING = 1,
  MOTOR_INIT_BOARD_CONFIGS_INVALID = 2,
  MOTOR_INIT_COMPONENTS_INIT_FAILED = 4,
  MOTOR_INIT_DRIVER_INIT_FAILED = 8,
  CURR_SENSE_INIT_FACTOR_OUT_OF_RANGE = 16,
  CURR_SENSE_CALIB_VALUE_OUT_OF_RANGE = 32,
  BUS_SENSE_CONFIGS_OUT_OF_RANGE = 64,
  I2C_FREQ_OUT_OF_RANGE = 128
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MISCONFIGURED_AREA_H