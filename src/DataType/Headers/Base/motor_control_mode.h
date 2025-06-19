/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/motor_control_mode.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_MOTOR_CONTROL_MODE_H
#define BASE_MOTOR_CONTROL_MODE_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorControlMode : uint32_t
{
  CTRL_MODE_POSITION = 0,
  CTRL_MODE_VELOCITY = 1,
  CTRL_MODE_CURRENT = 2,
  CTRL_MODE_HYBRID = 3
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MOTOR_CONTROL_MODE_H