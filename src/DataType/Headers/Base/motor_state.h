/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/motor_state.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_MOTOR_STATE_H
#define BASE_MOTOR_STATE_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorState : uint32_t
{
  IDLE = 0,
  STARTUP_SEQUENCE = 1,
  BASIC_PARAM_CALIBRATION = 2,
  ENCODER_INDEX_SEARCH = 3,
  ENCODER_CALIBRATION = 4,
  EXTEND_PARAM_CALIBRATION = 5,
  SENSORED_CLOSED_LOOP_CONTROL = 6,
  SENSORLESS_CLOSED_LOOP_CONTROL = 7
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MOTOR_STATE_H