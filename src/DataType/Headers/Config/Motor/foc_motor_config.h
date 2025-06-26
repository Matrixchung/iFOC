/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Config/Motor/foc_motor_config.proto
 */

// This file is generated. Please do not edit!
#ifndef CONFIG_MOTOR_FOC_MOTOR_CONFIG_H
#define CONFIG_MOTOR_FOC_MOTOR_CONFIG_H

#include <cstdint>
#include <MessageInterface.h>
#include "reflection.h"
#include <WireFormatter.h>
#include <Fields.h>
#include <MessageSizeCalculator.h>
#include <ReadBufferSection.h>
#include <RepeatedFieldFixedSize.h>
#include <FieldStringBytes.h>
#include <Errors.h>
#include <Defines.h>
#include <limits>

// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Config {
namespace Motor {

class FOCMotorConfig final: public ::EmbeddedProto::MessageInterface
{
  public:
        REFLECT(
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_encoder_index_search_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_sensored_closed_loop_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, calibration_voltage_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_basic_param_calibration_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, kv_rating_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, watchdog_timeout_sec_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, vel_kp_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, phase_inductance_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, max_current_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, pole_pairs_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, torque_constant_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_encoder_calibration_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, phase_inductance_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_sensorless_closed_loop_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, current_loop_bandwidth_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_extend_param_calibration_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, phase_resistance_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, flux_linkage_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, flux_linkage_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, sensor_direction_clockwise_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, pole_pairs_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, d_axis_inductance_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, q_axis_inductance_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, sensor_zero_offset_rad_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, startup_sequence_enabled_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, max_output_speed_rpm_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, calibration_current_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, sensor_zero_offset_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, deduction_ratio_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, sensor_direction_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, pos_kp_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, kv_rating_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, max_voltage_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, vel_ki_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, phase_resistance_valid_),
        MEMBER_SIZE_OFFSET(FOCMotorConfig, torque_constant_valid_)
    )
FOCMotorConfig() = default;
    FOCMotorConfig(const FOCMotorConfig& rhs )
    {
      set_current_loop_bandwidth(rhs.get_current_loop_bandwidth());
      set_calibration_voltage(rhs.get_calibration_voltage());
      set_calibration_current(rhs.get_calibration_current());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_phase_resistance(rhs.get_phase_resistance());
      set_phase_inductance(rhs.get_phase_inductance());
      set_q_axis_inductance(rhs.get_q_axis_inductance());
      set_d_axis_inductance(rhs.get_d_axis_inductance());
      set_pole_pairs(rhs.get_pole_pairs());
      set_sensor_zero_offset_rad(rhs.get_sensor_zero_offset_rad());
      set_kv_rating(rhs.get_kv_rating());
      set_torque_constant(rhs.get_torque_constant());
      set_flux_linkage(rhs.get_flux_linkage());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      set_phase_resistance_valid(rhs.get_phase_resistance_valid());
      set_phase_inductance_valid(rhs.get_phase_inductance_valid());
      set_sensor_direction_clockwise(rhs.get_sensor_direction_clockwise());
      set_sensor_direction_valid(rhs.get_sensor_direction_valid());
      set_pole_pairs_valid(rhs.get_pole_pairs_valid());
      set_sensor_zero_offset_valid(rhs.get_sensor_zero_offset_valid());
      set_kv_rating_valid(rhs.get_kv_rating_valid());
      set_torque_constant_valid(rhs.get_torque_constant_valid());
      set_flux_linkage_valid(rhs.get_flux_linkage_valid());
      set_startup_sequence_enabled(rhs.get_startup_sequence_enabled());
      set_startup_basic_param_calibration(rhs.get_startup_basic_param_calibration());
      set_startup_encoder_index_search(rhs.get_startup_encoder_index_search());
      set_startup_encoder_calibration(rhs.get_startup_encoder_calibration());
      set_startup_extend_param_calibration(rhs.get_startup_extend_param_calibration());
      set_startup_sensored_closed_loop(rhs.get_startup_sensored_closed_loop());
      set_startup_sensorless_closed_loop(rhs.get_startup_sensorless_closed_loop());
    }

    FOCMotorConfig(const FOCMotorConfig&& rhs ) noexcept
    {
      set_current_loop_bandwidth(rhs.get_current_loop_bandwidth());
      set_calibration_voltage(rhs.get_calibration_voltage());
      set_calibration_current(rhs.get_calibration_current());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_phase_resistance(rhs.get_phase_resistance());
      set_phase_inductance(rhs.get_phase_inductance());
      set_q_axis_inductance(rhs.get_q_axis_inductance());
      set_d_axis_inductance(rhs.get_d_axis_inductance());
      set_pole_pairs(rhs.get_pole_pairs());
      set_sensor_zero_offset_rad(rhs.get_sensor_zero_offset_rad());
      set_kv_rating(rhs.get_kv_rating());
      set_torque_constant(rhs.get_torque_constant());
      set_flux_linkage(rhs.get_flux_linkage());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      set_phase_resistance_valid(rhs.get_phase_resistance_valid());
      set_phase_inductance_valid(rhs.get_phase_inductance_valid());
      set_sensor_direction_clockwise(rhs.get_sensor_direction_clockwise());
      set_sensor_direction_valid(rhs.get_sensor_direction_valid());
      set_pole_pairs_valid(rhs.get_pole_pairs_valid());
      set_sensor_zero_offset_valid(rhs.get_sensor_zero_offset_valid());
      set_kv_rating_valid(rhs.get_kv_rating_valid());
      set_torque_constant_valid(rhs.get_torque_constant_valid());
      set_flux_linkage_valid(rhs.get_flux_linkage_valid());
      set_startup_sequence_enabled(rhs.get_startup_sequence_enabled());
      set_startup_basic_param_calibration(rhs.get_startup_basic_param_calibration());
      set_startup_encoder_index_search(rhs.get_startup_encoder_index_search());
      set_startup_encoder_calibration(rhs.get_startup_encoder_calibration());
      set_startup_extend_param_calibration(rhs.get_startup_extend_param_calibration());
      set_startup_sensored_closed_loop(rhs.get_startup_sensored_closed_loop());
      set_startup_sensorless_closed_loop(rhs.get_startup_sensorless_closed_loop());
    }

    ~FOCMotorConfig() override = default;

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      CURRENT_LOOP_BANDWIDTH = 2,
      CALIBRATION_VOLTAGE = 7,
      CALIBRATION_CURRENT = 8,
      MAX_VOLTAGE = 9,
      MAX_CURRENT = 10,
      PHASE_RESISTANCE = 11,
      PHASE_RESISTANCE_VALID = 12,
      PHASE_INDUCTANCE = 13,
      Q_AXIS_INDUCTANCE = 14,
      D_AXIS_INDUCTANCE = 15,
      PHASE_INDUCTANCE_VALID = 16,
      SENSOR_DIRECTION_CLOCKWISE = 17,
      SENSOR_DIRECTION_VALID = 18,
      POLE_PAIRS = 19,
      POLE_PAIRS_VALID = 20,
      SENSOR_ZERO_OFFSET_RAD = 21,
      SENSOR_ZERO_OFFSET_VALID = 22,
      KV_RATING = 23,
      KV_RATING_VALID = 24,
      TORQUE_CONSTANT = 25,
      TORQUE_CONSTANT_VALID = 26,
      FLUX_LINKAGE = 27,
      FLUX_LINKAGE_VALID = 28,
      DEDUCTION_RATIO = 29,
      MAX_OUTPUT_SPEED_RPM = 30,
      POS_KP = 31,
      VEL_KP = 32,
      VEL_KI = 33,
      WATCHDOG_TIMEOUT_SEC = 34,
      STARTUP_SEQUENCE_ENABLED = 37,
      STARTUP_BASIC_PARAM_CALIBRATION = 38,
      STARTUP_ENCODER_INDEX_SEARCH = 39,
      STARTUP_ENCODER_CALIBRATION = 40,
      STARTUP_EXTEND_PARAM_CALIBRATION = 41,
      STARTUP_SENSORED_CLOSED_LOOP = 42,
      STARTUP_SENSORLESS_CLOSED_LOOP = 43
    };

    FOCMotorConfig& operator=(const FOCMotorConfig& rhs)
    {
      set_current_loop_bandwidth(rhs.get_current_loop_bandwidth());
      set_calibration_voltage(rhs.get_calibration_voltage());
      set_calibration_current(rhs.get_calibration_current());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_phase_resistance(rhs.get_phase_resistance());
      set_phase_inductance(rhs.get_phase_inductance());
      set_q_axis_inductance(rhs.get_q_axis_inductance());
      set_d_axis_inductance(rhs.get_d_axis_inductance());
      set_pole_pairs(rhs.get_pole_pairs());
      set_sensor_zero_offset_rad(rhs.get_sensor_zero_offset_rad());
      set_kv_rating(rhs.get_kv_rating());
      set_torque_constant(rhs.get_torque_constant());
      set_flux_linkage(rhs.get_flux_linkage());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      set_phase_resistance_valid(rhs.get_phase_resistance_valid());
      set_phase_inductance_valid(rhs.get_phase_inductance_valid());
      set_sensor_direction_clockwise(rhs.get_sensor_direction_clockwise());
      set_sensor_direction_valid(rhs.get_sensor_direction_valid());
      set_pole_pairs_valid(rhs.get_pole_pairs_valid());
      set_sensor_zero_offset_valid(rhs.get_sensor_zero_offset_valid());
      set_kv_rating_valid(rhs.get_kv_rating_valid());
      set_torque_constant_valid(rhs.get_torque_constant_valid());
      set_flux_linkage_valid(rhs.get_flux_linkage_valid());
      set_startup_sequence_enabled(rhs.get_startup_sequence_enabled());
      set_startup_basic_param_calibration(rhs.get_startup_basic_param_calibration());
      set_startup_encoder_index_search(rhs.get_startup_encoder_index_search());
      set_startup_encoder_calibration(rhs.get_startup_encoder_calibration());
      set_startup_extend_param_calibration(rhs.get_startup_extend_param_calibration());
      set_startup_sensored_closed_loop(rhs.get_startup_sensored_closed_loop());
      set_startup_sensorless_closed_loop(rhs.get_startup_sensorless_closed_loop());
      return *this;
    }

    FOCMotorConfig& operator=(const FOCMotorConfig&& rhs) noexcept
    {
      set_current_loop_bandwidth(rhs.get_current_loop_bandwidth());
      set_calibration_voltage(rhs.get_calibration_voltage());
      set_calibration_current(rhs.get_calibration_current());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_phase_resistance(rhs.get_phase_resistance());
      set_phase_inductance(rhs.get_phase_inductance());
      set_q_axis_inductance(rhs.get_q_axis_inductance());
      set_d_axis_inductance(rhs.get_d_axis_inductance());
      set_pole_pairs(rhs.get_pole_pairs());
      set_sensor_zero_offset_rad(rhs.get_sensor_zero_offset_rad());
      set_kv_rating(rhs.get_kv_rating());
      set_torque_constant(rhs.get_torque_constant());
      set_flux_linkage(rhs.get_flux_linkage());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      set_phase_resistance_valid(rhs.get_phase_resistance_valid());
      set_phase_inductance_valid(rhs.get_phase_inductance_valid());
      set_sensor_direction_clockwise(rhs.get_sensor_direction_clockwise());
      set_sensor_direction_valid(rhs.get_sensor_direction_valid());
      set_pole_pairs_valid(rhs.get_pole_pairs_valid());
      set_sensor_zero_offset_valid(rhs.get_sensor_zero_offset_valid());
      set_kv_rating_valid(rhs.get_kv_rating_valid());
      set_torque_constant_valid(rhs.get_torque_constant_valid());
      set_flux_linkage_valid(rhs.get_flux_linkage_valid());
      set_startup_sequence_enabled(rhs.get_startup_sequence_enabled());
      set_startup_basic_param_calibration(rhs.get_startup_basic_param_calibration());
      set_startup_encoder_index_search(rhs.get_startup_encoder_index_search());
      set_startup_encoder_calibration(rhs.get_startup_encoder_calibration());
      set_startup_extend_param_calibration(rhs.get_startup_extend_param_calibration());
      set_startup_sensored_closed_loop(rhs.get_startup_sensored_closed_loop());
      set_startup_sensorless_closed_loop(rhs.get_startup_sensorless_closed_loop());
      return *this;
    }

    static constexpr char const* CURRENT_LOOP_BANDWIDTH_NAME = "current_loop_bandwidth";
    inline void clear_current_loop_bandwidth() { current_loop_bandwidth_.clear(); }
    inline void set_current_loop_bandwidth(const float& value) { current_loop_bandwidth_ = value; }
    inline void set_current_loop_bandwidth(const float&& value) { current_loop_bandwidth_ = value; }
    inline float& mutable_current_loop_bandwidth() { return current_loop_bandwidth_.get(); }
    inline const float& get_current_loop_bandwidth() const { return current_loop_bandwidth_.get(); }
    inline float current_loop_bandwidth() const { return current_loop_bandwidth_.get(); }

    static constexpr char const* CALIBRATION_VOLTAGE_NAME = "calibration_voltage";
    inline void clear_calibration_voltage() { calibration_voltage_.clear(); }
    inline void set_calibration_voltage(const float& value) { calibration_voltage_ = value; }
    inline void set_calibration_voltage(const float&& value) { calibration_voltage_ = value; }
    inline float& mutable_calibration_voltage() { return calibration_voltage_.get(); }
    inline const float& get_calibration_voltage() const { return calibration_voltage_.get(); }
    inline float calibration_voltage() const { return calibration_voltage_.get(); }

    static constexpr char const* CALIBRATION_CURRENT_NAME = "calibration_current";
    inline void clear_calibration_current() { calibration_current_.clear(); }
    inline void set_calibration_current(const float& value) { calibration_current_ = value; }
    inline void set_calibration_current(const float&& value) { calibration_current_ = value; }
    inline float& mutable_calibration_current() { return calibration_current_.get(); }
    inline const float& get_calibration_current() const { return calibration_current_.get(); }
    inline float calibration_current() const { return calibration_current_.get(); }

    static constexpr char const* MAX_VOLTAGE_NAME = "max_voltage";
    inline void clear_max_voltage() { max_voltage_.clear(); }
    inline void set_max_voltage(const float& value) { max_voltage_ = value; }
    inline void set_max_voltage(const float&& value) { max_voltage_ = value; }
    inline float& mutable_max_voltage() { return max_voltage_.get(); }
    inline const float& get_max_voltage() const { return max_voltage_.get(); }
    inline float max_voltage() const { return max_voltage_.get(); }

    static constexpr char const* MAX_CURRENT_NAME = "max_current";
    inline void clear_max_current() { max_current_.clear(); }
    inline void set_max_current(const float& value) { max_current_ = value; }
    inline void set_max_current(const float&& value) { max_current_ = value; }
    inline float& mutable_max_current() { return max_current_.get(); }
    inline const float& get_max_current() const { return max_current_.get(); }
    inline float max_current() const { return max_current_.get(); }

    static constexpr char const* PHASE_RESISTANCE_NAME = "phase_resistance";
    inline void clear_phase_resistance() { phase_resistance_.clear(); }
    inline void set_phase_resistance(const float& value) { phase_resistance_ = value; }
    inline void set_phase_resistance(const float&& value) { phase_resistance_ = value; }
    inline float& mutable_phase_resistance() { return phase_resistance_.get(); }
    inline const float& get_phase_resistance() const { return phase_resistance_.get(); }
    inline float phase_resistance() const { return phase_resistance_.get(); }

    static constexpr char const* PHASE_INDUCTANCE_NAME = "phase_inductance";
    inline void clear_phase_inductance() { phase_inductance_.clear(); }
    inline void set_phase_inductance(const float& value) { phase_inductance_ = value; }
    inline void set_phase_inductance(const float&& value) { phase_inductance_ = value; }
    inline float& mutable_phase_inductance() { return phase_inductance_.get(); }
    inline const float& get_phase_inductance() const { return phase_inductance_.get(); }
    inline float phase_inductance() const { return phase_inductance_.get(); }

    static constexpr char const* Q_AXIS_INDUCTANCE_NAME = "q_axis_inductance";
    inline void clear_q_axis_inductance() { q_axis_inductance_.clear(); }
    inline void set_q_axis_inductance(const float& value) { q_axis_inductance_ = value; }
    inline void set_q_axis_inductance(const float&& value) { q_axis_inductance_ = value; }
    inline float& mutable_q_axis_inductance() { return q_axis_inductance_.get(); }
    inline const float& get_q_axis_inductance() const { return q_axis_inductance_.get(); }
    inline float q_axis_inductance() const { return q_axis_inductance_.get(); }

    static constexpr char const* D_AXIS_INDUCTANCE_NAME = "d_axis_inductance";
    inline void clear_d_axis_inductance() { d_axis_inductance_.clear(); }
    inline void set_d_axis_inductance(const float& value) { d_axis_inductance_ = value; }
    inline void set_d_axis_inductance(const float&& value) { d_axis_inductance_ = value; }
    inline float& mutable_d_axis_inductance() { return d_axis_inductance_.get(); }
    inline const float& get_d_axis_inductance() const { return d_axis_inductance_.get(); }
    inline float d_axis_inductance() const { return d_axis_inductance_.get(); }

    static constexpr char const* POLE_PAIRS_NAME = "pole_pairs";
    inline void clear_pole_pairs() { pole_pairs_.clear(); }
    inline void set_pole_pairs(const uint32_t& value) { pole_pairs_ = value; }
    inline void set_pole_pairs(const uint32_t&& value) { pole_pairs_ = value; }
    inline uint32_t& mutable_pole_pairs() { return pole_pairs_.get(); }
    inline const uint32_t& get_pole_pairs() const { return pole_pairs_.get(); }
    inline uint32_t pole_pairs() const { return pole_pairs_.get(); }

    static constexpr char const* SENSOR_ZERO_OFFSET_RAD_NAME = "sensor_zero_offset_rad";
    inline void clear_sensor_zero_offset_rad() { sensor_zero_offset_rad_.clear(); }
    inline void set_sensor_zero_offset_rad(const float& value) { sensor_zero_offset_rad_ = value; }
    inline void set_sensor_zero_offset_rad(const float&& value) { sensor_zero_offset_rad_ = value; }
    inline float& mutable_sensor_zero_offset_rad() { return sensor_zero_offset_rad_.get(); }
    inline const float& get_sensor_zero_offset_rad() const { return sensor_zero_offset_rad_.get(); }
    inline float sensor_zero_offset_rad() const { return sensor_zero_offset_rad_.get(); }

    static constexpr char const* KV_RATING_NAME = "kv_rating";
    inline void clear_kv_rating() { kv_rating_.clear(); }
    inline void set_kv_rating(const float& value) { kv_rating_ = value; }
    inline void set_kv_rating(const float&& value) { kv_rating_ = value; }
    inline float& mutable_kv_rating() { return kv_rating_.get(); }
    inline const float& get_kv_rating() const { return kv_rating_.get(); }
    inline float kv_rating() const { return kv_rating_.get(); }

    static constexpr char const* TORQUE_CONSTANT_NAME = "torque_constant";
    inline void clear_torque_constant() { torque_constant_.clear(); }
    inline void set_torque_constant(const float& value) { torque_constant_ = value; }
    inline void set_torque_constant(const float&& value) { torque_constant_ = value; }
    inline float& mutable_torque_constant() { return torque_constant_.get(); }
    inline const float& get_torque_constant() const { return torque_constant_.get(); }
    inline float torque_constant() const { return torque_constant_.get(); }

    static constexpr char const* FLUX_LINKAGE_NAME = "flux_linkage";
    inline void clear_flux_linkage() { flux_linkage_.clear(); }
    inline void set_flux_linkage(const float& value) { flux_linkage_ = value; }
    inline void set_flux_linkage(const float&& value) { flux_linkage_ = value; }
    inline float& mutable_flux_linkage() { return flux_linkage_.get(); }
    inline const float& get_flux_linkage() const { return flux_linkage_.get(); }
    inline float flux_linkage() const { return flux_linkage_.get(); }

    static constexpr char const* DEDUCTION_RATIO_NAME = "deduction_ratio";
    inline void clear_deduction_ratio() { deduction_ratio_.clear(); }
    inline void set_deduction_ratio(const float& value) { deduction_ratio_ = value; }
    inline void set_deduction_ratio(const float&& value) { deduction_ratio_ = value; }
    inline float& mutable_deduction_ratio() { return deduction_ratio_.get(); }
    inline const float& get_deduction_ratio() const { return deduction_ratio_.get(); }
    inline float deduction_ratio() const { return deduction_ratio_.get(); }

    static constexpr char const* MAX_OUTPUT_SPEED_RPM_NAME = "max_output_speed_rpm";
    inline void clear_max_output_speed_rpm() { max_output_speed_rpm_.clear(); }
    inline void set_max_output_speed_rpm(const float& value) { max_output_speed_rpm_ = value; }
    inline void set_max_output_speed_rpm(const float&& value) { max_output_speed_rpm_ = value; }
    inline float& mutable_max_output_speed_rpm() { return max_output_speed_rpm_.get(); }
    inline const float& get_max_output_speed_rpm() const { return max_output_speed_rpm_.get(); }
    inline float max_output_speed_rpm() const { return max_output_speed_rpm_.get(); }

    static constexpr char const* POS_KP_NAME = "pos_kp";
    inline void clear_pos_kp() { pos_kp_.clear(); }
    inline void set_pos_kp(const float& value) { pos_kp_ = value; }
    inline void set_pos_kp(const float&& value) { pos_kp_ = value; }
    inline float& mutable_pos_kp() { return pos_kp_.get(); }
    inline const float& get_pos_kp() const { return pos_kp_.get(); }
    inline float pos_kp() const { return pos_kp_.get(); }

    static constexpr char const* VEL_KP_NAME = "vel_kp";
    inline void clear_vel_kp() { vel_kp_.clear(); }
    inline void set_vel_kp(const float& value) { vel_kp_ = value; }
    inline void set_vel_kp(const float&& value) { vel_kp_ = value; }
    inline float& mutable_vel_kp() { return vel_kp_.get(); }
    inline const float& get_vel_kp() const { return vel_kp_.get(); }
    inline float vel_kp() const { return vel_kp_.get(); }

    static constexpr char const* VEL_KI_NAME = "vel_ki";
    inline void clear_vel_ki() { vel_ki_.clear(); }
    inline void set_vel_ki(const float& value) { vel_ki_ = value; }
    inline void set_vel_ki(const float&& value) { vel_ki_ = value; }
    inline float& mutable_vel_ki() { return vel_ki_.get(); }
    inline const float& get_vel_ki() const { return vel_ki_.get(); }
    inline float vel_ki() const { return vel_ki_.get(); }

    static constexpr char const* WATCHDOG_TIMEOUT_SEC_NAME = "watchdog_timeout_sec";
    inline void clear_watchdog_timeout_sec() { watchdog_timeout_sec_.clear(); }
    inline void set_watchdog_timeout_sec(const float& value) { watchdog_timeout_sec_ = value; }
    inline void set_watchdog_timeout_sec(const float&& value) { watchdog_timeout_sec_ = value; }
    inline float& mutable_watchdog_timeout_sec() { return watchdog_timeout_sec_.get(); }
    inline const float& get_watchdog_timeout_sec() const { return watchdog_timeout_sec_.get(); }
    inline float watchdog_timeout_sec() const { return watchdog_timeout_sec_.get(); }

    static constexpr char const* PHASE_RESISTANCE_VALID_NAME = "phase_resistance_valid";
    inline void clear_phase_resistance_valid() { phase_resistance_valid_.clear(); }
    inline void set_phase_resistance_valid(const bool& value) { phase_resistance_valid_ = value; }
    inline void set_phase_resistance_valid(const bool&& value) { phase_resistance_valid_ = value; }
    inline bool& mutable_phase_resistance_valid() { return phase_resistance_valid_.get(); }
    inline const bool& get_phase_resistance_valid() const { return phase_resistance_valid_.get(); }
    inline bool phase_resistance_valid() const { return phase_resistance_valid_.get(); }

    static constexpr char const* PHASE_INDUCTANCE_VALID_NAME = "phase_inductance_valid";
    inline void clear_phase_inductance_valid() { phase_inductance_valid_.clear(); }
    inline void set_phase_inductance_valid(const bool& value) { phase_inductance_valid_ = value; }
    inline void set_phase_inductance_valid(const bool&& value) { phase_inductance_valid_ = value; }
    inline bool& mutable_phase_inductance_valid() { return phase_inductance_valid_.get(); }
    inline const bool& get_phase_inductance_valid() const { return phase_inductance_valid_.get(); }
    inline bool phase_inductance_valid() const { return phase_inductance_valid_.get(); }

    static constexpr char const* SENSOR_DIRECTION_CLOCKWISE_NAME = "sensor_direction_clockwise";
    inline void clear_sensor_direction_clockwise() { sensor_direction_clockwise_.clear(); }
    inline void set_sensor_direction_clockwise(const bool& value) { sensor_direction_clockwise_ = value; }
    inline void set_sensor_direction_clockwise(const bool&& value) { sensor_direction_clockwise_ = value; }
    inline bool& mutable_sensor_direction_clockwise() { return sensor_direction_clockwise_.get(); }
    inline const bool& get_sensor_direction_clockwise() const { return sensor_direction_clockwise_.get(); }
    inline bool sensor_direction_clockwise() const { return sensor_direction_clockwise_.get(); }

    static constexpr char const* SENSOR_DIRECTION_VALID_NAME = "sensor_direction_valid";
    inline void clear_sensor_direction_valid() { sensor_direction_valid_.clear(); }
    inline void set_sensor_direction_valid(const bool& value) { sensor_direction_valid_ = value; }
    inline void set_sensor_direction_valid(const bool&& value) { sensor_direction_valid_ = value; }
    inline bool& mutable_sensor_direction_valid() { return sensor_direction_valid_.get(); }
    inline const bool& get_sensor_direction_valid() const { return sensor_direction_valid_.get(); }
    inline bool sensor_direction_valid() const { return sensor_direction_valid_.get(); }

    static constexpr char const* POLE_PAIRS_VALID_NAME = "pole_pairs_valid";
    inline void clear_pole_pairs_valid() { pole_pairs_valid_.clear(); }
    inline void set_pole_pairs_valid(const bool& value) { pole_pairs_valid_ = value; }
    inline void set_pole_pairs_valid(const bool&& value) { pole_pairs_valid_ = value; }
    inline bool& mutable_pole_pairs_valid() { return pole_pairs_valid_.get(); }
    inline const bool& get_pole_pairs_valid() const { return pole_pairs_valid_.get(); }
    inline bool pole_pairs_valid() const { return pole_pairs_valid_.get(); }

    static constexpr char const* SENSOR_ZERO_OFFSET_VALID_NAME = "sensor_zero_offset_valid";
    inline void clear_sensor_zero_offset_valid() { sensor_zero_offset_valid_.clear(); }
    inline void set_sensor_zero_offset_valid(const bool& value) { sensor_zero_offset_valid_ = value; }
    inline void set_sensor_zero_offset_valid(const bool&& value) { sensor_zero_offset_valid_ = value; }
    inline bool& mutable_sensor_zero_offset_valid() { return sensor_zero_offset_valid_.get(); }
    inline const bool& get_sensor_zero_offset_valid() const { return sensor_zero_offset_valid_.get(); }
    inline bool sensor_zero_offset_valid() const { return sensor_zero_offset_valid_.get(); }

    static constexpr char const* KV_RATING_VALID_NAME = "kv_rating_valid";
    inline void clear_kv_rating_valid() { kv_rating_valid_.clear(); }
    inline void set_kv_rating_valid(const bool& value) { kv_rating_valid_ = value; }
    inline void set_kv_rating_valid(const bool&& value) { kv_rating_valid_ = value; }
    inline bool& mutable_kv_rating_valid() { return kv_rating_valid_.get(); }
    inline const bool& get_kv_rating_valid() const { return kv_rating_valid_.get(); }
    inline bool kv_rating_valid() const { return kv_rating_valid_.get(); }

    static constexpr char const* TORQUE_CONSTANT_VALID_NAME = "torque_constant_valid";
    inline void clear_torque_constant_valid() { torque_constant_valid_.clear(); }
    inline void set_torque_constant_valid(const bool& value) { torque_constant_valid_ = value; }
    inline void set_torque_constant_valid(const bool&& value) { torque_constant_valid_ = value; }
    inline bool& mutable_torque_constant_valid() { return torque_constant_valid_.get(); }
    inline const bool& get_torque_constant_valid() const { return torque_constant_valid_.get(); }
    inline bool torque_constant_valid() const { return torque_constant_valid_.get(); }

    static constexpr char const* FLUX_LINKAGE_VALID_NAME = "flux_linkage_valid";
    inline void clear_flux_linkage_valid() { flux_linkage_valid_.clear(); }
    inline void set_flux_linkage_valid(const bool& value) { flux_linkage_valid_ = value; }
    inline void set_flux_linkage_valid(const bool&& value) { flux_linkage_valid_ = value; }
    inline bool& mutable_flux_linkage_valid() { return flux_linkage_valid_.get(); }
    inline const bool& get_flux_linkage_valid() const { return flux_linkage_valid_.get(); }
    inline bool flux_linkage_valid() const { return flux_linkage_valid_.get(); }

    static constexpr char const* STARTUP_SEQUENCE_ENABLED_NAME = "startup_sequence_enabled";
    inline void clear_startup_sequence_enabled() { startup_sequence_enabled_.clear(); }
    inline void set_startup_sequence_enabled(const bool& value) { startup_sequence_enabled_ = value; }
    inline void set_startup_sequence_enabled(const bool&& value) { startup_sequence_enabled_ = value; }
    inline bool& mutable_startup_sequence_enabled() { return startup_sequence_enabled_.get(); }
    inline const bool& get_startup_sequence_enabled() const { return startup_sequence_enabled_.get(); }
    inline bool startup_sequence_enabled() const { return startup_sequence_enabled_.get(); }

    static constexpr char const* STARTUP_BASIC_PARAM_CALIBRATION_NAME = "startup_basic_param_calibration";
    inline void clear_startup_basic_param_calibration() { startup_basic_param_calibration_.clear(); }
    inline void set_startup_basic_param_calibration(const bool& value) { startup_basic_param_calibration_ = value; }
    inline void set_startup_basic_param_calibration(const bool&& value) { startup_basic_param_calibration_ = value; }
    inline bool& mutable_startup_basic_param_calibration() { return startup_basic_param_calibration_.get(); }
    inline const bool& get_startup_basic_param_calibration() const { return startup_basic_param_calibration_.get(); }
    inline bool startup_basic_param_calibration() const { return startup_basic_param_calibration_.get(); }

    static constexpr char const* STARTUP_ENCODER_INDEX_SEARCH_NAME = "startup_encoder_index_search";
    inline void clear_startup_encoder_index_search() { startup_encoder_index_search_.clear(); }
    inline void set_startup_encoder_index_search(const bool& value) { startup_encoder_index_search_ = value; }
    inline void set_startup_encoder_index_search(const bool&& value) { startup_encoder_index_search_ = value; }
    inline bool& mutable_startup_encoder_index_search() { return startup_encoder_index_search_.get(); }
    inline const bool& get_startup_encoder_index_search() const { return startup_encoder_index_search_.get(); }
    inline bool startup_encoder_index_search() const { return startup_encoder_index_search_.get(); }

    static constexpr char const* STARTUP_ENCODER_CALIBRATION_NAME = "startup_encoder_calibration";
    inline void clear_startup_encoder_calibration() { startup_encoder_calibration_.clear(); }
    inline void set_startup_encoder_calibration(const bool& value) { startup_encoder_calibration_ = value; }
    inline void set_startup_encoder_calibration(const bool&& value) { startup_encoder_calibration_ = value; }
    inline bool& mutable_startup_encoder_calibration() { return startup_encoder_calibration_.get(); }
    inline const bool& get_startup_encoder_calibration() const { return startup_encoder_calibration_.get(); }
    inline bool startup_encoder_calibration() const { return startup_encoder_calibration_.get(); }

    static constexpr char const* STARTUP_EXTEND_PARAM_CALIBRATION_NAME = "startup_extend_param_calibration";
    inline void clear_startup_extend_param_calibration() { startup_extend_param_calibration_.clear(); }
    inline void set_startup_extend_param_calibration(const bool& value) { startup_extend_param_calibration_ = value; }
    inline void set_startup_extend_param_calibration(const bool&& value) { startup_extend_param_calibration_ = value; }
    inline bool& mutable_startup_extend_param_calibration() { return startup_extend_param_calibration_.get(); }
    inline const bool& get_startup_extend_param_calibration() const { return startup_extend_param_calibration_.get(); }
    inline bool startup_extend_param_calibration() const { return startup_extend_param_calibration_.get(); }

    static constexpr char const* STARTUP_SENSORED_CLOSED_LOOP_NAME = "startup_sensored_closed_loop";
    inline void clear_startup_sensored_closed_loop() { startup_sensored_closed_loop_.clear(); }
    inline void set_startup_sensored_closed_loop(const bool& value) { startup_sensored_closed_loop_ = value; }
    inline void set_startup_sensored_closed_loop(const bool&& value) { startup_sensored_closed_loop_ = value; }
    inline bool& mutable_startup_sensored_closed_loop() { return startup_sensored_closed_loop_.get(); }
    inline const bool& get_startup_sensored_closed_loop() const { return startup_sensored_closed_loop_.get(); }
    inline bool startup_sensored_closed_loop() const { return startup_sensored_closed_loop_.get(); }

    static constexpr char const* STARTUP_SENSORLESS_CLOSED_LOOP_NAME = "startup_sensorless_closed_loop";
    inline void clear_startup_sensorless_closed_loop() { startup_sensorless_closed_loop_.clear(); }
    inline void set_startup_sensorless_closed_loop(const bool& value) { startup_sensorless_closed_loop_ = value; }
    inline void set_startup_sensorless_closed_loop(const bool&& value) { startup_sensorless_closed_loop_ = value; }
    inline bool& mutable_startup_sensorless_closed_loop() { return startup_sensorless_closed_loop_.get(); }
    inline const bool& get_startup_sensorless_closed_loop() const { return startup_sensorless_closed_loop_.get(); }
    inline bool startup_sensorless_closed_loop() const { return startup_sensorless_closed_loop_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if((0.0 != current_loop_bandwidth_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = current_loop_bandwidth_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURRENT_LOOP_BANDWIDTH), buffer, false);
      }

      if((0.0 != calibration_voltage_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = calibration_voltage_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CALIBRATION_VOLTAGE), buffer, false);
      }

      if((0.0 != calibration_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = calibration_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CALIBRATION_CURRENT), buffer, false);
      }

      if((0.0 != max_voltage_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_voltage_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_VOLTAGE), buffer, false);
      }

      if((0.0 != max_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_CURRENT), buffer, false);
      }

      if((0.0 != phase_resistance_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = phase_resistance_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PHASE_RESISTANCE), buffer, false);
      }

      if((0.0 != phase_inductance_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = phase_inductance_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PHASE_INDUCTANCE), buffer, false);
      }

      if((0.0 != q_axis_inductance_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = q_axis_inductance_.serialize_with_id(static_cast<uint32_t>(FieldNumber::Q_AXIS_INDUCTANCE), buffer, false);
      }

      if((0.0 != d_axis_inductance_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = d_axis_inductance_.serialize_with_id(static_cast<uint32_t>(FieldNumber::D_AXIS_INDUCTANCE), buffer, false);
      }

      if((0U != pole_pairs_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = pole_pairs_.serialize_with_id(static_cast<uint32_t>(FieldNumber::POLE_PAIRS), buffer, false);
      }

      if((0.0 != sensor_zero_offset_rad_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = sensor_zero_offset_rad_.serialize_with_id(static_cast<uint32_t>(FieldNumber::SENSOR_ZERO_OFFSET_RAD), buffer, false);
      }

      if((0.0 != kv_rating_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = kv_rating_.serialize_with_id(static_cast<uint32_t>(FieldNumber::KV_RATING), buffer, false);
      }

      if((0.0 != torque_constant_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = torque_constant_.serialize_with_id(static_cast<uint32_t>(FieldNumber::TORQUE_CONSTANT), buffer, false);
      }

      if((0.0 != flux_linkage_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = flux_linkage_.serialize_with_id(static_cast<uint32_t>(FieldNumber::FLUX_LINKAGE), buffer, false);
      }

      if((0.0 != deduction_ratio_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = deduction_ratio_.serialize_with_id(static_cast<uint32_t>(FieldNumber::DEDUCTION_RATIO), buffer, false);
      }

      if((0.0 != max_output_speed_rpm_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_output_speed_rpm_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_OUTPUT_SPEED_RPM), buffer, false);
      }

      if((0.0 != pos_kp_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = pos_kp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::POS_KP), buffer, false);
      }

      if((0.0 != vel_kp_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = vel_kp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::VEL_KP), buffer, false);
      }

      if((0.0 != vel_ki_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = vel_ki_.serialize_with_id(static_cast<uint32_t>(FieldNumber::VEL_KI), buffer, false);
      }

      if((0.0 != watchdog_timeout_sec_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = watchdog_timeout_sec_.serialize_with_id(static_cast<uint32_t>(FieldNumber::WATCHDOG_TIMEOUT_SEC), buffer, false);
      }

      if((false != phase_resistance_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = phase_resistance_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PHASE_RESISTANCE_VALID), buffer, false);
      }

      if((false != phase_inductance_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = phase_inductance_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PHASE_INDUCTANCE_VALID), buffer, false);
      }

      if((false != sensor_direction_clockwise_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = sensor_direction_clockwise_.serialize_with_id(static_cast<uint32_t>(FieldNumber::SENSOR_DIRECTION_CLOCKWISE), buffer, false);
      }

      if((false != sensor_direction_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = sensor_direction_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::SENSOR_DIRECTION_VALID), buffer, false);
      }

      if((false != pole_pairs_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = pole_pairs_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::POLE_PAIRS_VALID), buffer, false);
      }

      if((false != sensor_zero_offset_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = sensor_zero_offset_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::SENSOR_ZERO_OFFSET_VALID), buffer, false);
      }

      if((false != kv_rating_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = kv_rating_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::KV_RATING_VALID), buffer, false);
      }

      if((false != torque_constant_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = torque_constant_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::TORQUE_CONSTANT_VALID), buffer, false);
      }

      if((false != flux_linkage_valid_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = flux_linkage_valid_.serialize_with_id(static_cast<uint32_t>(FieldNumber::FLUX_LINKAGE_VALID), buffer, false);
      }

      if((false != startup_sequence_enabled_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_sequence_enabled_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_SEQUENCE_ENABLED), buffer, false);
      }

      if((false != startup_basic_param_calibration_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_basic_param_calibration_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_BASIC_PARAM_CALIBRATION), buffer, false);
      }

      if((false != startup_encoder_index_search_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_encoder_index_search_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_ENCODER_INDEX_SEARCH), buffer, false);
      }

      if((false != startup_encoder_calibration_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_encoder_calibration_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_ENCODER_CALIBRATION), buffer, false);
      }

      if((false != startup_extend_param_calibration_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_extend_param_calibration_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_EXTEND_PARAM_CALIBRATION), buffer, false);
      }

      if((false != startup_sensored_closed_loop_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_sensored_closed_loop_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_SENSORED_CLOSED_LOOP), buffer, false);
      }

      if((false != startup_sensorless_closed_loop_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = startup_sensorless_closed_loop_.serialize_with_id(static_cast<uint32_t>(FieldNumber::STARTUP_SENSORLESS_CLOSED_LOOP), buffer, false);
      }

      return return_value;
    };

    ::EmbeddedProto::Error deserialize(::EmbeddedProto::ReadBufferInterface& buffer) override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;
      ::EmbeddedProto::WireFormatter::WireType wire_type = ::EmbeddedProto::WireFormatter::WireType::VARINT;
      uint32_t id_number = 0;
      FieldNumber id_tag = FieldNumber::NOT_SET;

      ::EmbeddedProto::Error tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
      while((::EmbeddedProto::Error::NO_ERRORS == return_value) && (::EmbeddedProto::Error::NO_ERRORS == tag_value))
      {
        id_tag = static_cast<FieldNumber>(id_number);
        switch(id_tag)
        {
          case FieldNumber::CURRENT_LOOP_BANDWIDTH:
            return_value = current_loop_bandwidth_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CALIBRATION_VOLTAGE:
            return_value = calibration_voltage_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CALIBRATION_CURRENT:
            return_value = calibration_current_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_VOLTAGE:
            return_value = max_voltage_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_CURRENT:
            return_value = max_current_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PHASE_RESISTANCE:
            return_value = phase_resistance_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PHASE_INDUCTANCE:
            return_value = phase_inductance_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::Q_AXIS_INDUCTANCE:
            return_value = q_axis_inductance_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::D_AXIS_INDUCTANCE:
            return_value = d_axis_inductance_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::POLE_PAIRS:
            return_value = pole_pairs_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::SENSOR_ZERO_OFFSET_RAD:
            return_value = sensor_zero_offset_rad_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::KV_RATING:
            return_value = kv_rating_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::TORQUE_CONSTANT:
            return_value = torque_constant_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::FLUX_LINKAGE:
            return_value = flux_linkage_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::DEDUCTION_RATIO:
            return_value = deduction_ratio_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_OUTPUT_SPEED_RPM:
            return_value = max_output_speed_rpm_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::POS_KP:
            return_value = pos_kp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::VEL_KP:
            return_value = vel_kp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::VEL_KI:
            return_value = vel_ki_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::WATCHDOG_TIMEOUT_SEC:
            return_value = watchdog_timeout_sec_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PHASE_RESISTANCE_VALID:
            return_value = phase_resistance_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PHASE_INDUCTANCE_VALID:
            return_value = phase_inductance_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::SENSOR_DIRECTION_CLOCKWISE:
            return_value = sensor_direction_clockwise_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::SENSOR_DIRECTION_VALID:
            return_value = sensor_direction_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::POLE_PAIRS_VALID:
            return_value = pole_pairs_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::SENSOR_ZERO_OFFSET_VALID:
            return_value = sensor_zero_offset_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::KV_RATING_VALID:
            return_value = kv_rating_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::TORQUE_CONSTANT_VALID:
            return_value = torque_constant_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::FLUX_LINKAGE_VALID:
            return_value = flux_linkage_valid_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_SEQUENCE_ENABLED:
            return_value = startup_sequence_enabled_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_BASIC_PARAM_CALIBRATION:
            return_value = startup_basic_param_calibration_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_ENCODER_INDEX_SEARCH:
            return_value = startup_encoder_index_search_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_ENCODER_CALIBRATION:
            return_value = startup_encoder_calibration_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_EXTEND_PARAM_CALIBRATION:
            return_value = startup_extend_param_calibration_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_SENSORED_CLOSED_LOOP:
            return_value = startup_sensored_closed_loop_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::STARTUP_SENSORLESS_CLOSED_LOOP:
            return_value = startup_sensorless_closed_loop_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::NOT_SET:
            return_value = ::EmbeddedProto::Error::INVALID_FIELD_ID;
            break;

          default:
            return_value = skip_unknown_field(buffer, wire_type);
            break;
        }

        if(::EmbeddedProto::Error::NO_ERRORS == return_value)
        {
          // Read the next tag.
          tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
        }
      }

      // When an error was detect while reading the tag but no other errors where found, set it in the return value.
      if((::EmbeddedProto::Error::NO_ERRORS == return_value)
         && (::EmbeddedProto::Error::NO_ERRORS != tag_value)
         && (::EmbeddedProto::Error::END_OF_BUFFER != tag_value)) // The end of the buffer is not an array in this case.
      {
        return_value = tag_value;
      }

      return return_value;
    };

    void clear() override
    {
      clear_current_loop_bandwidth();
      clear_calibration_voltage();
      clear_calibration_current();
      clear_max_voltage();
      clear_max_current();
      clear_phase_resistance();
      clear_phase_inductance();
      clear_q_axis_inductance();
      clear_d_axis_inductance();
      clear_pole_pairs();
      clear_sensor_zero_offset_rad();
      clear_kv_rating();
      clear_torque_constant();
      clear_flux_linkage();
      clear_deduction_ratio();
      clear_max_output_speed_rpm();
      clear_pos_kp();
      clear_vel_kp();
      clear_vel_ki();
      clear_watchdog_timeout_sec();
      clear_phase_resistance_valid();
      clear_phase_inductance_valid();
      clear_sensor_direction_clockwise();
      clear_sensor_direction_valid();
      clear_pole_pairs_valid();
      clear_sensor_zero_offset_valid();
      clear_kv_rating_valid();
      clear_torque_constant_valid();
      clear_flux_linkage_valid();
      clear_startup_sequence_enabled();
      clear_startup_basic_param_calibration();
      clear_startup_encoder_index_search();
      clear_startup_encoder_calibration();
      clear_startup_extend_param_calibration();
      clear_startup_sensored_closed_loop();
      clear_startup_sensorless_closed_loop();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::CURRENT_LOOP_BANDWIDTH:
          name = CURRENT_LOOP_BANDWIDTH_NAME;
          break;
        case FieldNumber::CALIBRATION_VOLTAGE:
          name = CALIBRATION_VOLTAGE_NAME;
          break;
        case FieldNumber::CALIBRATION_CURRENT:
          name = CALIBRATION_CURRENT_NAME;
          break;
        case FieldNumber::MAX_VOLTAGE:
          name = MAX_VOLTAGE_NAME;
          break;
        case FieldNumber::MAX_CURRENT:
          name = MAX_CURRENT_NAME;
          break;
        case FieldNumber::PHASE_RESISTANCE:
          name = PHASE_RESISTANCE_NAME;
          break;
        case FieldNumber::PHASE_INDUCTANCE:
          name = PHASE_INDUCTANCE_NAME;
          break;
        case FieldNumber::Q_AXIS_INDUCTANCE:
          name = Q_AXIS_INDUCTANCE_NAME;
          break;
        case FieldNumber::D_AXIS_INDUCTANCE:
          name = D_AXIS_INDUCTANCE_NAME;
          break;
        case FieldNumber::POLE_PAIRS:
          name = POLE_PAIRS_NAME;
          break;
        case FieldNumber::SENSOR_ZERO_OFFSET_RAD:
          name = SENSOR_ZERO_OFFSET_RAD_NAME;
          break;
        case FieldNumber::KV_RATING:
          name = KV_RATING_NAME;
          break;
        case FieldNumber::TORQUE_CONSTANT:
          name = TORQUE_CONSTANT_NAME;
          break;
        case FieldNumber::FLUX_LINKAGE:
          name = FLUX_LINKAGE_NAME;
          break;
        case FieldNumber::DEDUCTION_RATIO:
          name = DEDUCTION_RATIO_NAME;
          break;
        case FieldNumber::MAX_OUTPUT_SPEED_RPM:
          name = MAX_OUTPUT_SPEED_RPM_NAME;
          break;
        case FieldNumber::POS_KP:
          name = POS_KP_NAME;
          break;
        case FieldNumber::VEL_KP:
          name = VEL_KP_NAME;
          break;
        case FieldNumber::VEL_KI:
          name = VEL_KI_NAME;
          break;
        case FieldNumber::WATCHDOG_TIMEOUT_SEC:
          name = WATCHDOG_TIMEOUT_SEC_NAME;
          break;
        case FieldNumber::PHASE_RESISTANCE_VALID:
          name = PHASE_RESISTANCE_VALID_NAME;
          break;
        case FieldNumber::PHASE_INDUCTANCE_VALID:
          name = PHASE_INDUCTANCE_VALID_NAME;
          break;
        case FieldNumber::SENSOR_DIRECTION_CLOCKWISE:
          name = SENSOR_DIRECTION_CLOCKWISE_NAME;
          break;
        case FieldNumber::SENSOR_DIRECTION_VALID:
          name = SENSOR_DIRECTION_VALID_NAME;
          break;
        case FieldNumber::POLE_PAIRS_VALID:
          name = POLE_PAIRS_VALID_NAME;
          break;
        case FieldNumber::SENSOR_ZERO_OFFSET_VALID:
          name = SENSOR_ZERO_OFFSET_VALID_NAME;
          break;
        case FieldNumber::KV_RATING_VALID:
          name = KV_RATING_VALID_NAME;
          break;
        case FieldNumber::TORQUE_CONSTANT_VALID:
          name = TORQUE_CONSTANT_VALID_NAME;
          break;
        case FieldNumber::FLUX_LINKAGE_VALID:
          name = FLUX_LINKAGE_VALID_NAME;
          break;
        case FieldNumber::STARTUP_SEQUENCE_ENABLED:
          name = STARTUP_SEQUENCE_ENABLED_NAME;
          break;
        case FieldNumber::STARTUP_BASIC_PARAM_CALIBRATION:
          name = STARTUP_BASIC_PARAM_CALIBRATION_NAME;
          break;
        case FieldNumber::STARTUP_ENCODER_INDEX_SEARCH:
          name = STARTUP_ENCODER_INDEX_SEARCH_NAME;
          break;
        case FieldNumber::STARTUP_ENCODER_CALIBRATION:
          name = STARTUP_ENCODER_CALIBRATION_NAME;
          break;
        case FieldNumber::STARTUP_EXTEND_PARAM_CALIBRATION:
          name = STARTUP_EXTEND_PARAM_CALIBRATION_NAME;
          break;
        case FieldNumber::STARTUP_SENSORED_CLOSED_LOOP:
          name = STARTUP_SENSORED_CLOSED_LOOP_NAME;
          break;
        case FieldNumber::STARTUP_SENSORLESS_CLOSED_LOOP:
          name = STARTUP_SENSORLESS_CLOSED_LOOP_NAME;
          break;
        default:
          name = "Invalid FieldNumber";
          break;
      }
      return name;
    }

#ifdef MSG_TO_STRING

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str) const
    {
      return this->to_string(str, 0, nullptr, true);
    }

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str, const uint32_t indent_level, char const* name, const bool first_field) const override
    {
      ::EmbeddedProto::string_view left_chars = str;
      int32_t n_chars_used = 0;

      if(!first_field)
      {
        // Add a comma behind the previous field.
        n_chars_used = snprintf(left_chars.data, left_chars.size, ",\n");
        if(0 < n_chars_used)
        {
          // Update the character pointer and characters left in the array.
          left_chars.data += n_chars_used;
          left_chars.size -= n_chars_used;
        }
      }

      if(nullptr != name)
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "\"%s\": {\n", name);
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s\"%s\": {\n", indent_level, " ", name);
        }
      }
      else
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "{\n");
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s{\n", indent_level, " ");
        }
      }
      
      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      left_chars = current_loop_bandwidth_.to_string(left_chars, indent_level + 2, CURRENT_LOOP_BANDWIDTH_NAME, true);
      left_chars = calibration_voltage_.to_string(left_chars, indent_level + 2, CALIBRATION_VOLTAGE_NAME, false);
      left_chars = calibration_current_.to_string(left_chars, indent_level + 2, CALIBRATION_CURRENT_NAME, false);
      left_chars = max_voltage_.to_string(left_chars, indent_level + 2, MAX_VOLTAGE_NAME, false);
      left_chars = max_current_.to_string(left_chars, indent_level + 2, MAX_CURRENT_NAME, false);
      left_chars = phase_resistance_.to_string(left_chars, indent_level + 2, PHASE_RESISTANCE_NAME, false);
      left_chars = phase_inductance_.to_string(left_chars, indent_level + 2, PHASE_INDUCTANCE_NAME, false);
      left_chars = q_axis_inductance_.to_string(left_chars, indent_level + 2, Q_AXIS_INDUCTANCE_NAME, false);
      left_chars = d_axis_inductance_.to_string(left_chars, indent_level + 2, D_AXIS_INDUCTANCE_NAME, false);
      left_chars = pole_pairs_.to_string(left_chars, indent_level + 2, POLE_PAIRS_NAME, false);
      left_chars = sensor_zero_offset_rad_.to_string(left_chars, indent_level + 2, SENSOR_ZERO_OFFSET_RAD_NAME, false);
      left_chars = kv_rating_.to_string(left_chars, indent_level + 2, KV_RATING_NAME, false);
      left_chars = torque_constant_.to_string(left_chars, indent_level + 2, TORQUE_CONSTANT_NAME, false);
      left_chars = flux_linkage_.to_string(left_chars, indent_level + 2, FLUX_LINKAGE_NAME, false);
      left_chars = deduction_ratio_.to_string(left_chars, indent_level + 2, DEDUCTION_RATIO_NAME, false);
      left_chars = max_output_speed_rpm_.to_string(left_chars, indent_level + 2, MAX_OUTPUT_SPEED_RPM_NAME, false);
      left_chars = pos_kp_.to_string(left_chars, indent_level + 2, POS_KP_NAME, false);
      left_chars = vel_kp_.to_string(left_chars, indent_level + 2, VEL_KP_NAME, false);
      left_chars = vel_ki_.to_string(left_chars, indent_level + 2, VEL_KI_NAME, false);
      left_chars = watchdog_timeout_sec_.to_string(left_chars, indent_level + 2, WATCHDOG_TIMEOUT_SEC_NAME, false);
      left_chars = phase_resistance_valid_.to_string(left_chars, indent_level + 2, PHASE_RESISTANCE_VALID_NAME, false);
      left_chars = phase_inductance_valid_.to_string(left_chars, indent_level + 2, PHASE_INDUCTANCE_VALID_NAME, false);
      left_chars = sensor_direction_clockwise_.to_string(left_chars, indent_level + 2, SENSOR_DIRECTION_CLOCKWISE_NAME, false);
      left_chars = sensor_direction_valid_.to_string(left_chars, indent_level + 2, SENSOR_DIRECTION_VALID_NAME, false);
      left_chars = pole_pairs_valid_.to_string(left_chars, indent_level + 2, POLE_PAIRS_VALID_NAME, false);
      left_chars = sensor_zero_offset_valid_.to_string(left_chars, indent_level + 2, SENSOR_ZERO_OFFSET_VALID_NAME, false);
      left_chars = kv_rating_valid_.to_string(left_chars, indent_level + 2, KV_RATING_VALID_NAME, false);
      left_chars = torque_constant_valid_.to_string(left_chars, indent_level + 2, TORQUE_CONSTANT_VALID_NAME, false);
      left_chars = flux_linkage_valid_.to_string(left_chars, indent_level + 2, FLUX_LINKAGE_VALID_NAME, false);
      left_chars = startup_sequence_enabled_.to_string(left_chars, indent_level + 2, STARTUP_SEQUENCE_ENABLED_NAME, false);
      left_chars = startup_basic_param_calibration_.to_string(left_chars, indent_level + 2, STARTUP_BASIC_PARAM_CALIBRATION_NAME, false);
      left_chars = startup_encoder_index_search_.to_string(left_chars, indent_level + 2, STARTUP_ENCODER_INDEX_SEARCH_NAME, false);
      left_chars = startup_encoder_calibration_.to_string(left_chars, indent_level + 2, STARTUP_ENCODER_CALIBRATION_NAME, false);
      left_chars = startup_extend_param_calibration_.to_string(left_chars, indent_level + 2, STARTUP_EXTEND_PARAM_CALIBRATION_NAME, false);
      left_chars = startup_sensored_closed_loop_.to_string(left_chars, indent_level + 2, STARTUP_SENSORED_CLOSED_LOOP_NAME, false);
      left_chars = startup_sensorless_closed_loop_.to_string(left_chars, indent_level + 2, STARTUP_SENSORLESS_CLOSED_LOOP_NAME, false);
  
      if( 0 == indent_level) 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n}");
      }
      else 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n%*s}", indent_level, " ");
      }

      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      return left_chars;
    }

#endif // End of MSG_TO_STRING

  private:


      EmbeddedProto::floatfixed current_loop_bandwidth_ = 0.0;
      EmbeddedProto::floatfixed calibration_voltage_ = 0.0;
      EmbeddedProto::floatfixed calibration_current_ = 0.0;
      EmbeddedProto::floatfixed max_voltage_ = 0.0;
      EmbeddedProto::floatfixed max_current_ = 0.0;
      EmbeddedProto::floatfixed phase_resistance_ = 0.0;
      EmbeddedProto::floatfixed phase_inductance_ = 0.0;
      EmbeddedProto::floatfixed q_axis_inductance_ = 0.0;
      EmbeddedProto::floatfixed d_axis_inductance_ = 0.0;
      EmbeddedProto::uint32 pole_pairs_ = 0U;
      EmbeddedProto::floatfixed sensor_zero_offset_rad_ = 0.0;
      EmbeddedProto::floatfixed kv_rating_ = 0.0;
      EmbeddedProto::floatfixed torque_constant_ = 0.0;
      EmbeddedProto::floatfixed flux_linkage_ = 0.0;
      EmbeddedProto::floatfixed deduction_ratio_ = 0.0;
      EmbeddedProto::floatfixed max_output_speed_rpm_ = 0.0;
      EmbeddedProto::floatfixed pos_kp_ = 0.0;
      EmbeddedProto::floatfixed vel_kp_ = 0.0;
      EmbeddedProto::floatfixed vel_ki_ = 0.0;
      EmbeddedProto::floatfixed watchdog_timeout_sec_ = 0.0;
      EmbeddedProto::boolean phase_resistance_valid_ = false;
      EmbeddedProto::boolean phase_inductance_valid_ = false;
      EmbeddedProto::boolean sensor_direction_clockwise_ = false;
      EmbeddedProto::boolean sensor_direction_valid_ = false;
      EmbeddedProto::boolean pole_pairs_valid_ = false;
      EmbeddedProto::boolean sensor_zero_offset_valid_ = false;
      EmbeddedProto::boolean kv_rating_valid_ = false;
      EmbeddedProto::boolean torque_constant_valid_ = false;
      EmbeddedProto::boolean flux_linkage_valid_ = false;
      EmbeddedProto::boolean startup_sequence_enabled_ = false;
      EmbeddedProto::boolean startup_basic_param_calibration_ = false;
      EmbeddedProto::boolean startup_encoder_index_search_ = false;
      EmbeddedProto::boolean startup_encoder_calibration_ = false;
      EmbeddedProto::boolean startup_extend_param_calibration_ = false;
      EmbeddedProto::boolean startup_sensored_closed_loop_ = false;
      EmbeddedProto::boolean startup_sensorless_closed_loop_ = false;

};

} // End of namespace Motor
} // End of namespace Config
} // End of namespace DataType
} // End of namespace iFOC
#endif // CONFIG_MOTOR_FOC_MOTOR_CONFIG_H