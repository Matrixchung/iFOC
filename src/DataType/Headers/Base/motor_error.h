/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/motor_error.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_MOTOR_ERROR_H
#define BASE_MOTOR_ERROR_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorError : uint32_t
{
  NONE = 0,
  _NON_CRITICAL_ERROR_ABOVE_ = 1024,
  PHASE_IMBALANCE = 1025,
  PHASE_RESISTANCE_OUT_OF_RANGE = 1026,
  PHASE_INDUCTANCE_OUT_OF_RANGE = 1027,
  MOTOR_FAILED_TO_ROTATE = 1028,
  POLE_PAIR_NUMBER_OUT_OF_RANGE = 1029,
  DC_BUS_OVER_DRAIN_CURRENT = 1030,
  DC_BUS_OVER_RECHARGE_CURRENT = 1031,
  PHASE_D_Q_AXIS_OVER_CURRENT = 1032,
  MOTOR_CONFIG_SAVE_FAILED = 1035,
  PWM_WAVE_FREQUENCY_OUT_OF_RANGE = 1036,
  NO_REGISTERED_VALID_SENSOR = 1040,
  PRIMARY_SENSOR_RESULT_INVALID = 1041,
  STARTUP_SENSORED_CLOSE_LOOP_REQ_NOT_MET = 1042,
  STARTUP_SENSORLESS_CLOSE_LOOP_REQ_NOT_MET = 1043,
  SYSTEM_CRITICAL_MALLOC_FAILED = 1098,
  SYSTEM_CRITICAL_TASK_STACK_OVERFLOW = 1099
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MOTOR_ERROR_H