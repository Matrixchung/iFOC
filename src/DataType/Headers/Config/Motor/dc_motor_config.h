#pragma once

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

#include "dc_motor_config.pb.h"
#include "../../../reflection.h"

namespace iFOC {
namespace DataType {
namespace Config {
namespace Motor {

class DCMotorConfig
{
public:
    DCMotorConfig_data _d{};   /* MUST be first data member for reflection */

    DCMotorConfig() = default;

    REFLECT(
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, node_id),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, max_voltage),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, max_current),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, deduction_ratio),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, max_output_speed_rpm),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, pos_kp),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, vel_kp),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, vel_ki),
        MEMBER_SIZE_OFFSET(DCMotorConfig_data, watchdog_timeout_sec)
    )

    static const pb_msgdesc_t* pb_fields() { return DCMotorConfig_data_fields; }
    const void* pb_data() const             { return &_d; }
    void*       pb_data()                   { return &_d; }
    void        clear()                     { memset(&_d, 0, sizeof(_d)); }

    /* ── Inline getters / setters (zero code-size overhead) ── */
    uint32_t node_id()                      const { return _d.node_id; }
    uint32_t get_node_id()                   const { return _d.node_id; }
    void set_node_id(uint32_t v)                     { _d.node_id = v; }
    float max_voltage()                  const { return _d.max_voltage; }
    float get_max_voltage()               const { return _d.max_voltage; }
    void set_max_voltage(float v)                 { _d.max_voltage = v; }
    float max_current()                  const { return _d.max_current; }
    float get_max_current()               const { return _d.max_current; }
    void set_max_current(float v)                 { _d.max_current = v; }
    float deduction_ratio()              const { return _d.deduction_ratio; }
    float get_deduction_ratio()           const { return _d.deduction_ratio; }
    void set_deduction_ratio(float v)             { _d.deduction_ratio = v; }
    float max_output_speed_rpm()         const { return _d.max_output_speed_rpm; }
    float get_max_output_speed_rpm()      const { return _d.max_output_speed_rpm; }
    void set_max_output_speed_rpm(float v)        { _d.max_output_speed_rpm = v; }
    float pos_kp()                       const { return _d.pos_kp; }
    float get_pos_kp()                    const { return _d.pos_kp; }
    void set_pos_kp(float v)                      { _d.pos_kp = v; }
    float vel_kp()                       const { return _d.vel_kp; }
    float get_vel_kp()                    const { return _d.vel_kp; }
    void set_vel_kp(float v)                      { _d.vel_kp = v; }
    float vel_ki()                       const { return _d.vel_ki; }
    float get_vel_ki()                    const { return _d.vel_ki; }
    void set_vel_ki(float v)                      { _d.vel_ki = v; }
    float watchdog_timeout_sec()         const { return _d.watchdog_timeout_sec; }
    float get_watchdog_timeout_sec()      const { return _d.watchdog_timeout_sec; }
    void set_watchdog_timeout_sec(float v)        { _d.watchdog_timeout_sec = v; }
};

} // namespace Motor
} // namespace Config
} // namespace DataType
} // namespace iFOC

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
