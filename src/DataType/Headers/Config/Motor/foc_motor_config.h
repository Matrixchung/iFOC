#pragma once

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

#include "foc_motor_config.pb.h"
#include "../../../reflection.h"

namespace iFOC {
namespace DataType {
namespace Config {
namespace Motor {

class FOCMotorConfig
{
public:
    FOCMotorConfig_data _d{};   /* MUST be first data member for reflection */

    FOCMotorConfig() = default;

    REFLECT(
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, node_id),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, current_loop_bandwidth),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, calibration_voltage),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, calibration_current),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, max_voltage),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, max_current),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, phase_resistance),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, phase_resistance_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, phase_inductance),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, q_axis_inductance),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, d_axis_inductance),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, phase_inductance_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, sensor_direction_clockwise),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, sensor_direction_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, pole_pairs),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, pole_pairs_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, sensor_zero_offset_rad),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, sensor_zero_offset_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, kv_rating),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, kv_rating_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, torque_constant),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, torque_constant_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, flux_linkage),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, flux_linkage_valid),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, deduction_ratio),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, max_output_speed_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, pos_kp),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, vel_kp),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, vel_ki),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, watchdog_timeout_sec),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, sensor_speed_f_lp),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_sequence_enabled),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_basic_param_calibration),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_encoder_index_search),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_encoder_calibration),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_extend_param_calibration),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_sensored_closed_loop),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, startup_sensorless_closed_loop),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, can_heartbeat_interval_ms),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, can_feedback_interval_ms),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, can_misc_fdbk_interval_ms),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, enable_harmonic_suppression),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, enable_anticogging),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, anticogging_base_pos_err_deg),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, anticogging_base_vel_err_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, traj_output_speed_limit_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, traj_output_accel_limit_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, traj_output_decel_limit_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, mit_output_pos_range_deg),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, mit_output_vel_range_rpm),
        MEMBER_SIZE_OFFSET(FOCMotorConfig_data, mit_output_tor_range_nm)
    )

    static const pb_msgdesc_t* pb_fields() { return FOCMotorConfig_data_fields; }
    const void* pb_data() const             { return &_d; }
    void*       pb_data()                   { return &_d; }
    void        clear()                     { memset(&_d, 0, sizeof(_d)); }

    /* ── Inline getters / setters (zero code-size overhead) ── */
    uint32_t node_id()                                  const { return _d.node_id; }
    uint32_t get_node_id()                               const { return _d.node_id; }
    void set_node_id(uint32_t v)                                 { _d.node_id = v; }
    float current_loop_bandwidth()                   const { return _d.current_loop_bandwidth; }
    float get_current_loop_bandwidth()                const { return _d.current_loop_bandwidth; }
    void set_current_loop_bandwidth(float v)                  { _d.current_loop_bandwidth = v; }
    /* node_name: not in REFLECT (STRING type), max 32 chars */
    const char* node_name()                          const { return _d.node_name; }
    const char* get_node_name()                       const { return _d.node_name; }
    void set_node_name(const char* v)                        { strncpy(_d.node_name, v, sizeof(_d.node_name) - 1); _d.node_name[sizeof(_d.node_name) - 1] = '\0'; }
    float calibration_voltage()                      const { return _d.calibration_voltage; }
    float get_calibration_voltage()                   const { return _d.calibration_voltage; }
    void set_calibration_voltage(float v)                     { _d.calibration_voltage = v; }
    float calibration_current()                      const { return _d.calibration_current; }
    float get_calibration_current()                   const { return _d.calibration_current; }
    void set_calibration_current(float v)                     { _d.calibration_current = v; }
    float max_voltage()                              const { return _d.max_voltage; }
    float get_max_voltage()                           const { return _d.max_voltage; }
    void set_max_voltage(float v)                             { _d.max_voltage = v; }
    float max_current()                              const { return _d.max_current; }
    float get_max_current()                           const { return _d.max_current; }
    void set_max_current(float v)                             { _d.max_current = v; }
    float phase_resistance()                         const { return _d.phase_resistance; }
    float get_phase_resistance()                      const { return _d.phase_resistance; }
    void set_phase_resistance(float v)                        { _d.phase_resistance = v; }
    bool phase_resistance_valid()                   const { return _d.phase_resistance_valid; }
    bool get_phase_resistance_valid()                const { return _d.phase_resistance_valid; }
    void set_phase_resistance_valid(bool v)                  { _d.phase_resistance_valid = v; }
    float phase_inductance()                         const { return _d.phase_inductance; }
    float get_phase_inductance()                      const { return _d.phase_inductance; }
    void set_phase_inductance(float v)                        { _d.phase_inductance = v; }
    float q_axis_inductance()                        const { return _d.q_axis_inductance; }
    float get_q_axis_inductance()                     const { return _d.q_axis_inductance; }
    void set_q_axis_inductance(float v)                       { _d.q_axis_inductance = v; }
    float d_axis_inductance()                        const { return _d.d_axis_inductance; }
    float get_d_axis_inductance()                     const { return _d.d_axis_inductance; }
    void set_d_axis_inductance(float v)                       { _d.d_axis_inductance = v; }
    bool phase_inductance_valid()                   const { return _d.phase_inductance_valid; }
    bool get_phase_inductance_valid()                const { return _d.phase_inductance_valid; }
    void set_phase_inductance_valid(bool v)                  { _d.phase_inductance_valid = v; }
    bool sensor_direction_clockwise()               const { return _d.sensor_direction_clockwise; }
    bool get_sensor_direction_clockwise()            const { return _d.sensor_direction_clockwise; }
    void set_sensor_direction_clockwise(bool v)              { _d.sensor_direction_clockwise = v; }
    bool sensor_direction_valid()                   const { return _d.sensor_direction_valid; }
    bool get_sensor_direction_valid()                const { return _d.sensor_direction_valid; }
    void set_sensor_direction_valid(bool v)                  { _d.sensor_direction_valid = v; }
    uint32_t pole_pairs()                               const { return _d.pole_pairs; }
    uint32_t get_pole_pairs()                            const { return _d.pole_pairs; }
    void set_pole_pairs(uint32_t v)                              { _d.pole_pairs = v; }
    bool pole_pairs_valid()                         const { return _d.pole_pairs_valid; }
    bool get_pole_pairs_valid()                      const { return _d.pole_pairs_valid; }
    void set_pole_pairs_valid(bool v)                        { _d.pole_pairs_valid = v; }
    float sensor_zero_offset_rad()                   const { return _d.sensor_zero_offset_rad; }
    float get_sensor_zero_offset_rad()                const { return _d.sensor_zero_offset_rad; }
    void set_sensor_zero_offset_rad(float v)                  { _d.sensor_zero_offset_rad = v; }
    bool sensor_zero_offset_valid()                 const { return _d.sensor_zero_offset_valid; }
    bool get_sensor_zero_offset_valid()              const { return _d.sensor_zero_offset_valid; }
    void set_sensor_zero_offset_valid(bool v)                { _d.sensor_zero_offset_valid = v; }
    float kv_rating()                                const { return _d.kv_rating; }
    float get_kv_rating()                             const { return _d.kv_rating; }
    void set_kv_rating(float v)                               { _d.kv_rating = v; }
    bool kv_rating_valid()                          const { return _d.kv_rating_valid; }
    bool get_kv_rating_valid()                       const { return _d.kv_rating_valid; }
    void set_kv_rating_valid(bool v)                         { _d.kv_rating_valid = v; }
    float torque_constant()                          const { return _d.torque_constant; }
    float get_torque_constant()                       const { return _d.torque_constant; }
    void set_torque_constant(float v)                         { _d.torque_constant = v; }
    bool torque_constant_valid()                    const { return _d.torque_constant_valid; }
    bool get_torque_constant_valid()                 const { return _d.torque_constant_valid; }
    void set_torque_constant_valid(bool v)                   { _d.torque_constant_valid = v; }
    float flux_linkage()                             const { return _d.flux_linkage; }
    float get_flux_linkage()                          const { return _d.flux_linkage; }
    void set_flux_linkage(float v)                            { _d.flux_linkage = v; }
    bool flux_linkage_valid()                       const { return _d.flux_linkage_valid; }
    bool get_flux_linkage_valid()                    const { return _d.flux_linkage_valid; }
    void set_flux_linkage_valid(bool v)                      { _d.flux_linkage_valid = v; }
    float deduction_ratio()                          const { return _d.deduction_ratio; }
    float get_deduction_ratio()                       const { return _d.deduction_ratio; }
    void set_deduction_ratio(float v)                         { _d.deduction_ratio = v; }
    float max_output_speed_rpm()                     const { return _d.max_output_speed_rpm; }
    float get_max_output_speed_rpm()                  const { return _d.max_output_speed_rpm; }
    void set_max_output_speed_rpm(float v)                    { _d.max_output_speed_rpm = v; }
    float pos_kp()                                   const { return _d.pos_kp; }
    float get_pos_kp()                                const { return _d.pos_kp; }
    void set_pos_kp(float v)                                  { _d.pos_kp = v; }
    float vel_kp()                                   const { return _d.vel_kp; }
    float get_vel_kp()                                const { return _d.vel_kp; }
    void set_vel_kp(float v)                                  { _d.vel_kp = v; }
    float vel_ki()                                   const { return _d.vel_ki; }
    float get_vel_ki()                                const { return _d.vel_ki; }
    void set_vel_ki(float v)                                  { _d.vel_ki = v; }
    float watchdog_timeout_sec()                     const { return _d.watchdog_timeout_sec; }
    float get_watchdog_timeout_sec()                  const { return _d.watchdog_timeout_sec; }
    void set_watchdog_timeout_sec(float v)                    { _d.watchdog_timeout_sec = v; }
    float sensor_speed_f_lp()                        const { return _d.sensor_speed_f_lp; }
    float get_sensor_speed_f_lp()                     const { return _d.sensor_speed_f_lp; }
    void set_sensor_speed_f_lp(float v)                       { _d.sensor_speed_f_lp = v; }
    bool startup_sequence_enabled()                 const { return _d.startup_sequence_enabled; }
    bool get_startup_sequence_enabled()              const { return _d.startup_sequence_enabled; }
    void set_startup_sequence_enabled(bool v)                { _d.startup_sequence_enabled = v; }
    bool startup_basic_param_calibration()          const { return _d.startup_basic_param_calibration; }
    bool get_startup_basic_param_calibration()       const { return _d.startup_basic_param_calibration; }
    void set_startup_basic_param_calibration(bool v)         { _d.startup_basic_param_calibration = v; }
    bool startup_encoder_index_search()             const { return _d.startup_encoder_index_search; }
    bool get_startup_encoder_index_search()          const { return _d.startup_encoder_index_search; }
    void set_startup_encoder_index_search(bool v)            { _d.startup_encoder_index_search = v; }
    bool startup_encoder_calibration()              const { return _d.startup_encoder_calibration; }
    bool get_startup_encoder_calibration()           const { return _d.startup_encoder_calibration; }
    void set_startup_encoder_calibration(bool v)             { _d.startup_encoder_calibration = v; }
    bool startup_extend_param_calibration()         const { return _d.startup_extend_param_calibration; }
    bool get_startup_extend_param_calibration()      const { return _d.startup_extend_param_calibration; }
    void set_startup_extend_param_calibration(bool v)        { _d.startup_extend_param_calibration = v; }
    bool startup_sensored_closed_loop()             const { return _d.startup_sensored_closed_loop; }
    bool get_startup_sensored_closed_loop()          const { return _d.startup_sensored_closed_loop; }
    void set_startup_sensored_closed_loop(bool v)            { _d.startup_sensored_closed_loop = v; }
    bool startup_sensorless_closed_loop()           const { return _d.startup_sensorless_closed_loop; }
    bool get_startup_sensorless_closed_loop()        const { return _d.startup_sensorless_closed_loop; }
    void set_startup_sensorless_closed_loop(bool v)          { _d.startup_sensorless_closed_loop = v; }
    uint32_t can_heartbeat_interval_ms()                const { return _d.can_heartbeat_interval_ms; }
    uint32_t get_can_heartbeat_interval_ms()             const { return _d.can_heartbeat_interval_ms; }
    void set_can_heartbeat_interval_ms(uint32_t v)               { _d.can_heartbeat_interval_ms = v; }
    uint32_t can_feedback_interval_ms()                 const { return _d.can_feedback_interval_ms; }
    uint32_t get_can_feedback_interval_ms()              const { return _d.can_feedback_interval_ms; }
    void set_can_feedback_interval_ms(uint32_t v)                { _d.can_feedback_interval_ms = v; }
    uint32_t can_misc_fdbk_interval_ms()                const { return _d.can_misc_fdbk_interval_ms; }
    uint32_t get_can_misc_fdbk_interval_ms()             const { return _d.can_misc_fdbk_interval_ms; }
    void set_can_misc_fdbk_interval_ms(uint32_t v)               { _d.can_misc_fdbk_interval_ms = v; }
    bool enable_harmonic_suppression()              const { return _d.enable_harmonic_suppression; }
    bool get_enable_harmonic_suppression()           const { return _d.enable_harmonic_suppression; }
    void set_enable_harmonic_suppression(bool v)             { _d.enable_harmonic_suppression = v; }
    bool enable_anticogging()                       const { return _d.enable_anticogging; }
    bool get_enable_anticogging()                    const { return _d.enable_anticogging; }
    void set_enable_anticogging(bool v)                      { _d.enable_anticogging = v; }
    float anticogging_base_pos_err_deg()             const { return _d.anticogging_base_pos_err_deg; }
    float get_anticogging_base_pos_err_deg()          const { return _d.anticogging_base_pos_err_deg; }
    void set_anticogging_base_pos_err_deg(float v)            { _d.anticogging_base_pos_err_deg = v; }
    float anticogging_base_vel_err_rpm()             const { return _d.anticogging_base_vel_err_rpm; }
    float get_anticogging_base_vel_err_rpm()          const { return _d.anticogging_base_vel_err_rpm; }
    void set_anticogging_base_vel_err_rpm(float v)            { _d.anticogging_base_vel_err_rpm = v; }
    float traj_output_speed_limit_rpm()              const { return _d.traj_output_speed_limit_rpm; }
    float get_traj_output_speed_limit_rpm()           const { return _d.traj_output_speed_limit_rpm; }
    void set_traj_output_speed_limit_rpm(float v)             { _d.traj_output_speed_limit_rpm = v; }
    float traj_output_accel_limit_rpm()              const { return _d.traj_output_accel_limit_rpm; }
    float get_traj_output_accel_limit_rpm()           const { return _d.traj_output_accel_limit_rpm; }
    void set_traj_output_accel_limit_rpm(float v)             { _d.traj_output_accel_limit_rpm = v; }
    float traj_output_decel_limit_rpm()              const { return _d.traj_output_decel_limit_rpm; }
    float get_traj_output_decel_limit_rpm()           const { return _d.traj_output_decel_limit_rpm; }
    void set_traj_output_decel_limit_rpm(float v)             { _d.traj_output_decel_limit_rpm = v; }
    float mit_output_pos_range_deg()                 const { return _d.mit_output_pos_range_deg; }
    float get_mit_output_pos_range_deg()              const { return _d.mit_output_pos_range_deg; }
    void set_mit_output_pos_range_deg(float v)                { _d.mit_output_pos_range_deg = v; }
    float mit_output_vel_range_rpm()                 const { return _d.mit_output_vel_range_rpm; }
    float get_mit_output_vel_range_rpm()              const { return _d.mit_output_vel_range_rpm; }
    void set_mit_output_vel_range_rpm(float v)                { _d.mit_output_vel_range_rpm = v; }
    float mit_output_tor_range_nm()                  const { return _d.mit_output_tor_range_nm; }
    float get_mit_output_tor_range_nm()               const { return _d.mit_output_tor_range_nm; }
    void set_mit_output_tor_range_nm(float v)                 { _d.mit_output_tor_range_nm = v; }
};

} // namespace Motor
} // namespace Config
} // namespace DataType
} // namespace iFOC

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
