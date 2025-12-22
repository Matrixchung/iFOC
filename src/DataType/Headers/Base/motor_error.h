#ifndef BASE_MOTOR_ERROR_H
#define BASE_MOTOR_ERROR_H

#include <cstdint>

namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorError : uint64_t
{
  NONE = 0,

  // Motor-related errors
  MOTOR_PHASE_IMBALANCE               = (1LL << 0),
  MOTOR_PHASE_RESISTANCE_OUT_OF_RANGE = (1LL << 1),
  MOTOR_PHASE_INDUCTANCE_OUT_OF_RANGE = (1LL << 2),
  MOTOR_FAILED_TO_ROTATE              = (1LL << 3),
  MOTOR_POLE_PAIR_NUMBER_OUT_OF_RANGE = (1LL << 4),
  MOTOR_DC_BUS_OVER_DRAIN_CURRENT     = (1LL << 5),
  MOTOR_DC_BUS_OVER_RECHARGE_CURRENT  = (1LL << 6),
  MOTOR_PHASE_D_Q_AXIS_OVER_CURRENT   = (1LL << 7),
  MOTOR_OVERSPEED_PROTECTION          = (1LL << 8),
  MOTOR_OVERTEMP_PROTECTION           = (1LL << 9),
  MOTOR_MOSFET_OVERTEMP_PROTECTION    = (1LL << 10),

  // Startup sequence
  STARTUP_SEQ_REQUIREMENTS_UNMET      = (1LL << 11),

  // Sensor
  PRIMARY_SENSOR_COMPONENT_MISSING    = (1LL << 12),
  PRIMARY_SENSOR_INIT_FAILED          = (1LL << 13),
  PRIMARY_SENSOR_RESULT_INVALID       = (1LL << 14),
  PRIMARY_SENSOR_CALIBRATION_FAILED   = (1LL << 15),
  AUXILIARY_SENSOR_INIT_FAILED        = (1LL << 16),
  AUXILIARY_SENSOR_RESULT_INVALID     = (1LL << 17),
  AUXILIARY_SENSOR_CALIBRATION_FAILED = (1LL << 18),

  // System-related errors
  SYSTEM_MEM_ALLOCATION_FAILED        = (1LL << 19),
  SYSTEM_OS_TASK_STACK_OVERFLOW       = (1LL << 20),
  SYSTEM_RT_WATCHDOG_TIMEOUT          = (1LL << 21),
  SYSTEM_MID_WATCHDOG_TIMEOUT         = (1LL << 22),
  SYSTEM_OS_WATCHDOG_TIMEOUT          = (1LL << 23),
  SYSTEM_BOARD_CONFIG_READ_ERROR      = (1LL << 24),
  SYSTEM_BOARD_CONFIG_WRITE_ERROR     = (1LL << 25),
  SYSTEM_MOTOR_CONFIG_READ_ERROR      = (1LL << 26),
  SYSTEM_MOTOR_CONFIG_WRITE_ERROR     = (1LL << 27),
  SYSTEM_CORE_OVERTEMP                = (1LL << 28),

  // Configuration-related errors
  CONFIG_BOARD_CONFIG_INVALID         = (1LL << 29),
  CONFIG_CURR_SENSE_CONFIG_INVALID    = (1LL << 30),
  CONFIG_BUS_SENSE_CONFIG_INVALID     = (1LL << 31),

  // Current sense
  CURR_SENSE_COMPONENT_MISSING        = (1LL << 32),
  CURR_SENSE_INIT_FAILED              = (1LL << 33),
  CURR_SENSE_RESULT_INVALID           = (1LL << 34),

  // Bus sense
  BUS_SENSE_COMPONENT_MISSING         = (1LL << 35),
  BUS_SENSE_INIT_FAILED               = (1LL << 36),
  BUS_SENSE_DEV_ID_MISMATCH           = (1LL << 37),
  BUS_SENSE_RESULT_INVALID            = (1LL << 38),

  // Driver
  DRIVER_COMPONENT_MISSING            = (1LL << 39),
  DRIVER_INIT_FAILED                  = (1LL << 40),
  DRIVER_COMMUNICATION_ERROR          = (1LL << 41),
  DRIVER_DEV_ID_MISMATCH              = (1LL << 42),
  DRIVER_GATE_OVERCURRENT             = (1LL << 43),
  DRIVER_UNDERVOLTAGE_LOCKOUT         = (1LL << 44),
  DRIVER_OVERVOLTAGE_LOCKOUT          = (1LL << 45),
  DRIVER_OVERTEMP_PROTECTION          = (1LL << 46),

};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MOTOR_ERROR_H