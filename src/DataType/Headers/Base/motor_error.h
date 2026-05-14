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
  MOTOR_PHASE_IMBALANCE                = (1ULL << 0),
  MOTOR_PHASE_IMBALANCE_U              = (1ULL << 1),
  MOTOR_PHASE_IMBALANCE_V              = (1ULL << 2),
  MOTOR_PHASE_IMBALANCE_W              = (1ULL << 3),
  MOTOR_PHASE_RESISTANCE_OUT_OF_RANGE  = (1ULL << 4),
  MOTOR_PHASE_INDUCTANCE_OUT_OF_RANGE  = (1ULL << 5),
  MOTOR_FAILED_TO_ROTATE               = (1ULL << 6),
  MOTOR_POLE_PAIR_NUMBER_OUT_OF_RANGE  = (1ULL << 7),
  MOTOR_DC_BUS_OVER_DRAIN_CURRENT      = (1ULL << 8),
  MOTOR_DC_BUS_OVER_RECHARGE_CURRENT   = (1ULL << 9),
  MOTOR_DC_BUS_OVERVOLTAGE             = (1ULL << 10),
  MOTOR_DC_BUS_UNDERVOLTAGE            = (1ULL << 11),
  MOTOR_PHASE_D_Q_AXIS_OVER_CURRENT    = (1ULL << 12),
  MOTOR_OVERSPEED_PROTECTION           = (1ULL << 13),
  MOTOR_OVERTEMP_PROTECTION            = (1ULL << 14),
  MOTOR_MOSFET_OVERTEMP_PROTECTION     = (1ULL << 15),
  MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT = (1ULL << 16),

  // Startup sequence
  STARTUP_SEQ_REQUIREMENTS_UNMET       = (1ULL << 17),

  // Sensor
  PRIMARY_SENSOR_COMPONENT_MISSING     = (1ULL << 18),
  PRIMARY_SENSOR_INIT_FAILED           = (1ULL << 19),
  PRIMARY_SENSOR_RESULT_INVALID        = (1ULL << 20),
  PRIMARY_SENSOR_CALIBRATION_FAILED    = (1ULL << 21),
  AUXILIARY_SENSOR_INIT_FAILED         = (1ULL << 22),
  AUXILIARY_SENSOR_RESULT_INVALID      = (1ULL << 23),
  AUXILIARY_SENSOR_CALIBRATION_FAILED  = (1ULL << 24),

  // System-related errors
  SYSTEM_MEM_ALLOCATION_FAILED         = (1ULL << 25),
  SYSTEM_OS_TASK_STACK_OVERFLOW        = (1ULL << 26),
  SYSTEM_RT_WATCHDOG_TIMEOUT           = (1ULL << 27),
  SYSTEM_MID_WATCHDOG_TIMEOUT          = (1ULL << 28),
  SYSTEM_OS_WATCHDOG_TIMEOUT           = (1ULL << 29),
  SYSTEM_BOARD_CONFIG_READ_ERROR       = (1ULL << 30),
  SYSTEM_BOARD_CONFIG_WRITE_ERROR      = (1ULL << 31),
  SYSTEM_MOTOR_CONFIG_READ_ERROR       = (1ULL << 32),
  SYSTEM_MOTOR_CONFIG_WRITE_ERROR      = (1ULL << 33),
  SYSTEM_CORE_OVERTEMP                 = (1ULL << 34),

  // Configuration-related errors
  CONFIG_BOARD_CONFIG_INVALID          = (1ULL << 35),
  CONFIG_CURR_SENSE_CONFIG_INVALID     = (1ULL << 36),
  CONFIG_BUS_SENSE_CONFIG_INVALID      = (1ULL << 37),

  // Current sense
  CURR_SENSE_COMPONENT_MISSING         = (1ULL << 38),
  CURR_SENSE_INIT_FAILED               = (1ULL << 39),
  CURR_SENSE_RESULT_INVALID            = (1ULL << 40),

  // Bus sense
  BUS_SENSE_COMPONENT_MISSING          = (1ULL << 41),
  BUS_SENSE_INIT_FAILED                = (1ULL << 42),
  BUS_SENSE_DEV_ID_MISMATCH            = (1ULL << 43),
  BUS_SENSE_RESULT_INVALID             = (1ULL << 44),

  // Driver
  DRIVER_COMPONENT_MISSING             = (1ULL << 45),
  DRIVER_INIT_FAILED                   = (1ULL << 46),
  DRIVER_COMMUNICATION_ERROR           = (1ULL << 47),
  DRIVER_DEV_ID_MISMATCH               = (1ULL << 48),
  DRIVER_GATE_OVERCURRENT              = (1ULL << 49),
  DRIVER_UNDERVOLTAGE_LOCKOUT          = (1ULL << 50),
  DRIVER_OVERVOLTAGE_LOCKOUT           = (1ULL << 51),
  DRIVER_OVERTEMP_PROTECTION           = (1ULL << 52),

  // Anticogging stuffs
  ANTICOGGING_POS_UNSTABLE             = (1ULL << 53),
};

constexpr inline MotorError operator&(MotorError lhs, MotorError rhs) noexcept
{
  return static_cast<MotorError>(
      static_cast<std::underlying_type_t<MotorError>>(lhs) &
      static_cast<std::underlying_type_t<MotorError>>(rhs)
  );
}

constexpr inline MotorError operator|(MotorError lhs, MotorError rhs) noexcept
{
    return static_cast<MotorError>(
        static_cast<std::underlying_type_t<MotorError>>(lhs) |
        static_cast<std::underlying_type_t<MotorError>>(rhs)
    );
}

constexpr inline MotorError operator^(MotorError lhs, MotorError rhs) noexcept
{
    return static_cast<MotorError>(
        static_cast<std::underlying_type_t<MotorError>>(lhs) ^
        static_cast<std::underlying_type_t<MotorError>>(rhs)
    );
}

constexpr inline MotorError operator~(MotorError value) noexcept
{
    return static_cast<MotorError>(
        ~static_cast<std::underlying_type_t<MotorError>>(value)
    );
}

constexpr inline MotorError& operator&=(MotorError& lhs, MotorError rhs) noexcept
{
    lhs = lhs & rhs;
    return lhs;
}

constexpr inline MotorError& operator|=(MotorError& lhs, MotorError rhs) noexcept
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr inline MotorError& operator^=(MotorError& lhs, MotorError rhs) noexcept
{
    lhs = lhs ^ rhs;
    return lhs;
}

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_MOTOR_ERROR_H