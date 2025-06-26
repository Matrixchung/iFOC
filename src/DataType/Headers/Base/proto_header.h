/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/proto_header.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_PROTO_HEADER_H
#define BASE_PROTO_HEADER_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class ProtoHeader : uint32_t
{
  NOT_USED = 0,
  BOARD_CONFIG = 1,
  FOC_MOTOR_CONFIG_M1 = 2,
  FOC_MOTOR_CONFIG_M2 = 3,
  FOC_MOTOR_CONFIG_M3 = 4,
  FOC_MOTOR_CONFIG_M4 = 5,
  FOC_MOTOR_CONFIG_M5 = 6,
  FOC_MOTOR_CONFIG_M6 = 7,
  FOC_MOTOR_CONFIG_M7 = 8,
  FOC_MOTOR_CONFIG_M8 = 9,
  DC_MOTOR_CONFIG_M1 = 12,
  DC_MOTOR_CONFIG_M2 = 13,
  DC_MOTOR_CONFIG_M3 = 14,
  DC_MOTOR_CONFIG_M4 = 15,
  DC_MOTOR_CONFIG_M5 = 16,
  DC_MOTOR_CONFIG_M6 = 17,
  DC_MOTOR_CONFIG_M7 = 18,
  DC_MOTOR_CONFIG_M8 = 19
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_PROTO_HEADER_H