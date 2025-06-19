/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Config/Board/board_config.proto
 */

// This file is generated. Please do not edit!
#ifndef CONFIG_BOARD_BOARD_CONFIG_H
#define CONFIG_BOARD_BOARD_CONFIG_H

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

class BoardConfig final: public ::EmbeddedProto::MessageInterface
{
  public:
        REFLECT(
        MEMBER_SIZE_OFFSET(BoardConfig, bus_max_positive_current_),
        MEMBER_SIZE_OFFSET(BoardConfig, current_sense_dir_reversed_),
        MEMBER_SIZE_OFFSET(BoardConfig, uart_1_baudrate_),
        MEMBER_SIZE_OFFSET(BoardConfig, uart_1_protocol_),
        MEMBER_SIZE_OFFSET(BoardConfig, max_regen_current_),
        MEMBER_SIZE_OFFSET(BoardConfig, current_sense_f_lp_),
        MEMBER_SIZE_OFFSET(BoardConfig, bus_max_negative_current_),
        MEMBER_SIZE_OFFSET(BoardConfig, current_sense_shunt_ohm_),
        MEMBER_SIZE_OFFSET(BoardConfig, bus_sense_shunt_ohm_),
        MEMBER_SIZE_OFFSET(BoardConfig, bus_sense_dir_reversed_),
        MEMBER_SIZE_OFFSET(BoardConfig, current_sense_gain_),
        MEMBER_SIZE_OFFSET(BoardConfig, speed_loop_freq_),
        MEMBER_SIZE_OFFSET(BoardConfig, bus_overvoltage_limit_),
        MEMBER_SIZE_OFFSET(BoardConfig, play_startup_tone_),
        MEMBER_SIZE_OFFSET(BoardConfig, bus_undervoltage_limit_),
        MEMBER_SIZE_OFFSET(BoardConfig, pwm_wave_freq_)
    )
BoardConfig() = default;
    BoardConfig(const BoardConfig& rhs )
    {
      set_uart_1_protocol(rhs.get_uart_1_protocol());
      set_uart_1_baudrate(rhs.get_uart_1_baudrate());
      set_pwm_wave_freq(rhs.get_pwm_wave_freq());
      set_speed_loop_freq(rhs.get_speed_loop_freq());
      set_bus_overvoltage_limit(rhs.get_bus_overvoltage_limit());
      set_bus_undervoltage_limit(rhs.get_bus_undervoltage_limit());
      set_bus_max_positive_current(rhs.get_bus_max_positive_current());
      set_bus_max_negative_current(rhs.get_bus_max_negative_current());
      set_bus_sense_shunt_ohm(rhs.get_bus_sense_shunt_ohm());
      set_bus_sense_dir_reversed(rhs.get_bus_sense_dir_reversed());
      set_max_regen_current(rhs.get_max_regen_current());
      set_current_sense_gain(rhs.get_current_sense_gain());
      set_current_sense_shunt_ohm(rhs.get_current_sense_shunt_ohm());
      set_current_sense_dir_reversed(rhs.get_current_sense_dir_reversed());
      set_current_sense_f_lp(rhs.get_current_sense_f_lp());
      set_play_startup_tone(rhs.get_play_startup_tone());
    }

    BoardConfig(const BoardConfig&& rhs ) noexcept
    {
      set_uart_1_protocol(rhs.get_uart_1_protocol());
      set_uart_1_baudrate(rhs.get_uart_1_baudrate());
      set_pwm_wave_freq(rhs.get_pwm_wave_freq());
      set_speed_loop_freq(rhs.get_speed_loop_freq());
      set_bus_overvoltage_limit(rhs.get_bus_overvoltage_limit());
      set_bus_undervoltage_limit(rhs.get_bus_undervoltage_limit());
      set_bus_max_positive_current(rhs.get_bus_max_positive_current());
      set_bus_max_negative_current(rhs.get_bus_max_negative_current());
      set_bus_sense_shunt_ohm(rhs.get_bus_sense_shunt_ohm());
      set_bus_sense_dir_reversed(rhs.get_bus_sense_dir_reversed());
      set_max_regen_current(rhs.get_max_regen_current());
      set_current_sense_gain(rhs.get_current_sense_gain());
      set_current_sense_shunt_ohm(rhs.get_current_sense_shunt_ohm());
      set_current_sense_dir_reversed(rhs.get_current_sense_dir_reversed());
      set_current_sense_f_lp(rhs.get_current_sense_f_lp());
      set_play_startup_tone(rhs.get_play_startup_tone());
    }

    ~BoardConfig() override = default;

    enum class UARTProtocol : uint32_t
    {
      ASCII = 0,
      VOFA = 1,
      RAW_PROTO = 2,
      MATLAB = 3
    };

    enum class UARTBaudrate : uint32_t
    {
      BAUD_115200 = 0,
      BAUD_9600 = 1,
      BAUD_230400 = 2,
      BAUD_460800 = 3,
      BAUD_921600 = 4,
      BAUD_1843200 = 5
    };

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      UART_1_PROTOCOL = 1,
      UART_1_BAUDRATE = 2,
      PWM_WAVE_FREQ = 5,
      SPEED_LOOP_FREQ = 6,
      BUS_OVERVOLTAGE_LIMIT = 11,
      BUS_UNDERVOLTAGE_LIMIT = 12,
      BUS_MAX_POSITIVE_CURRENT = 13,
      BUS_MAX_NEGATIVE_CURRENT = 14,
      BUS_SENSE_SHUNT_OHM = 15,
      BUS_SENSE_DIR_REVERSED = 16,
      MAX_REGEN_CURRENT = 19,
      CURRENT_SENSE_GAIN = 21,
      CURRENT_SENSE_SHUNT_OHM = 22,
      CURRENT_SENSE_DIR_REVERSED = 23,
      CURRENT_SENSE_F_LP = 24,
      PLAY_STARTUP_TONE = 31
    };

    BoardConfig& operator=(const BoardConfig& rhs)
    {
      set_uart_1_protocol(rhs.get_uart_1_protocol());
      set_uart_1_baudrate(rhs.get_uart_1_baudrate());
      set_pwm_wave_freq(rhs.get_pwm_wave_freq());
      set_speed_loop_freq(rhs.get_speed_loop_freq());
      set_bus_overvoltage_limit(rhs.get_bus_overvoltage_limit());
      set_bus_undervoltage_limit(rhs.get_bus_undervoltage_limit());
      set_bus_max_positive_current(rhs.get_bus_max_positive_current());
      set_bus_max_negative_current(rhs.get_bus_max_negative_current());
      set_bus_sense_shunt_ohm(rhs.get_bus_sense_shunt_ohm());
      set_bus_sense_dir_reversed(rhs.get_bus_sense_dir_reversed());
      set_max_regen_current(rhs.get_max_regen_current());
      set_current_sense_gain(rhs.get_current_sense_gain());
      set_current_sense_shunt_ohm(rhs.get_current_sense_shunt_ohm());
      set_current_sense_dir_reversed(rhs.get_current_sense_dir_reversed());
      set_current_sense_f_lp(rhs.get_current_sense_f_lp());
      set_play_startup_tone(rhs.get_play_startup_tone());
      return *this;
    }

    BoardConfig& operator=(const BoardConfig&& rhs) noexcept
    {
      set_uart_1_protocol(rhs.get_uart_1_protocol());
      set_uart_1_baudrate(rhs.get_uart_1_baudrate());
      set_pwm_wave_freq(rhs.get_pwm_wave_freq());
      set_speed_loop_freq(rhs.get_speed_loop_freq());
      set_bus_overvoltage_limit(rhs.get_bus_overvoltage_limit());
      set_bus_undervoltage_limit(rhs.get_bus_undervoltage_limit());
      set_bus_max_positive_current(rhs.get_bus_max_positive_current());
      set_bus_max_negative_current(rhs.get_bus_max_negative_current());
      set_bus_sense_shunt_ohm(rhs.get_bus_sense_shunt_ohm());
      set_bus_sense_dir_reversed(rhs.get_bus_sense_dir_reversed());
      set_max_regen_current(rhs.get_max_regen_current());
      set_current_sense_gain(rhs.get_current_sense_gain());
      set_current_sense_shunt_ohm(rhs.get_current_sense_shunt_ohm());
      set_current_sense_dir_reversed(rhs.get_current_sense_dir_reversed());
      set_current_sense_f_lp(rhs.get_current_sense_f_lp());
      set_play_startup_tone(rhs.get_play_startup_tone());
      return *this;
    }

    static constexpr char const* UART_1_PROTOCOL_NAME = "uart_1_protocol";
    inline void clear_uart_1_protocol() { uart_1_protocol_.clear(); }
    inline void set_uart_1_protocol(const UARTProtocol& value) { uart_1_protocol_ = value; }
    inline void set_uart_1_protocol(const UARTProtocol&& value) { uart_1_protocol_ = value; }
    inline const UARTProtocol& get_uart_1_protocol() const { return uart_1_protocol_.get(); }
    inline UARTProtocol uart_1_protocol() const { return uart_1_protocol_.get(); }

    static constexpr char const* UART_1_BAUDRATE_NAME = "uart_1_baudrate";
    inline void clear_uart_1_baudrate() { uart_1_baudrate_.clear(); }
    inline void set_uart_1_baudrate(const UARTBaudrate& value) { uart_1_baudrate_ = value; }
    inline void set_uart_1_baudrate(const UARTBaudrate&& value) { uart_1_baudrate_ = value; }
    inline const UARTBaudrate& get_uart_1_baudrate() const { return uart_1_baudrate_.get(); }
    inline UARTBaudrate uart_1_baudrate() const { return uart_1_baudrate_.get(); }

    static constexpr char const* PWM_WAVE_FREQ_NAME = "pwm_wave_freq";
    inline void clear_pwm_wave_freq() { pwm_wave_freq_.clear(); }
    inline void set_pwm_wave_freq(const uint32_t& value) { pwm_wave_freq_ = value; }
    inline void set_pwm_wave_freq(const uint32_t&& value) { pwm_wave_freq_ = value; }
    inline uint32_t& mutable_pwm_wave_freq() { return pwm_wave_freq_.get(); }
    inline const uint32_t& get_pwm_wave_freq() const { return pwm_wave_freq_.get(); }
    inline uint32_t pwm_wave_freq() const { return pwm_wave_freq_.get(); }

    static constexpr char const* SPEED_LOOP_FREQ_NAME = "speed_loop_freq";
    inline void clear_speed_loop_freq() { speed_loop_freq_.clear(); }
    inline void set_speed_loop_freq(const uint32_t& value) { speed_loop_freq_ = value; }
    inline void set_speed_loop_freq(const uint32_t&& value) { speed_loop_freq_ = value; }
    inline uint32_t& mutable_speed_loop_freq() { return speed_loop_freq_.get(); }
    inline const uint32_t& get_speed_loop_freq() const { return speed_loop_freq_.get(); }
    inline uint32_t speed_loop_freq() const { return speed_loop_freq_.get(); }

    static constexpr char const* BUS_OVERVOLTAGE_LIMIT_NAME = "bus_overvoltage_limit";
    inline void clear_bus_overvoltage_limit() { bus_overvoltage_limit_.clear(); }
    inline void set_bus_overvoltage_limit(const float& value) { bus_overvoltage_limit_ = value; }
    inline void set_bus_overvoltage_limit(const float&& value) { bus_overvoltage_limit_ = value; }
    inline float& mutable_bus_overvoltage_limit() { return bus_overvoltage_limit_.get(); }
    inline const float& get_bus_overvoltage_limit() const { return bus_overvoltage_limit_.get(); }
    inline float bus_overvoltage_limit() const { return bus_overvoltage_limit_.get(); }

    static constexpr char const* BUS_UNDERVOLTAGE_LIMIT_NAME = "bus_undervoltage_limit";
    inline void clear_bus_undervoltage_limit() { bus_undervoltage_limit_.clear(); }
    inline void set_bus_undervoltage_limit(const float& value) { bus_undervoltage_limit_ = value; }
    inline void set_bus_undervoltage_limit(const float&& value) { bus_undervoltage_limit_ = value; }
    inline float& mutable_bus_undervoltage_limit() { return bus_undervoltage_limit_.get(); }
    inline const float& get_bus_undervoltage_limit() const { return bus_undervoltage_limit_.get(); }
    inline float bus_undervoltage_limit() const { return bus_undervoltage_limit_.get(); }

    static constexpr char const* BUS_MAX_POSITIVE_CURRENT_NAME = "bus_max_positive_current";
    inline void clear_bus_max_positive_current() { bus_max_positive_current_.clear(); }
    inline void set_bus_max_positive_current(const float& value) { bus_max_positive_current_ = value; }
    inline void set_bus_max_positive_current(const float&& value) { bus_max_positive_current_ = value; }
    inline float& mutable_bus_max_positive_current() { return bus_max_positive_current_.get(); }
    inline const float& get_bus_max_positive_current() const { return bus_max_positive_current_.get(); }
    inline float bus_max_positive_current() const { return bus_max_positive_current_.get(); }

    static constexpr char const* BUS_MAX_NEGATIVE_CURRENT_NAME = "bus_max_negative_current";
    inline void clear_bus_max_negative_current() { bus_max_negative_current_.clear(); }
    inline void set_bus_max_negative_current(const float& value) { bus_max_negative_current_ = value; }
    inline void set_bus_max_negative_current(const float&& value) { bus_max_negative_current_ = value; }
    inline float& mutable_bus_max_negative_current() { return bus_max_negative_current_.get(); }
    inline const float& get_bus_max_negative_current() const { return bus_max_negative_current_.get(); }
    inline float bus_max_negative_current() const { return bus_max_negative_current_.get(); }

    static constexpr char const* BUS_SENSE_SHUNT_OHM_NAME = "bus_sense_shunt_ohm";
    inline void clear_bus_sense_shunt_ohm() { bus_sense_shunt_ohm_.clear(); }
    inline void set_bus_sense_shunt_ohm(const float& value) { bus_sense_shunt_ohm_ = value; }
    inline void set_bus_sense_shunt_ohm(const float&& value) { bus_sense_shunt_ohm_ = value; }
    inline float& mutable_bus_sense_shunt_ohm() { return bus_sense_shunt_ohm_.get(); }
    inline const float& get_bus_sense_shunt_ohm() const { return bus_sense_shunt_ohm_.get(); }
    inline float bus_sense_shunt_ohm() const { return bus_sense_shunt_ohm_.get(); }

    static constexpr char const* BUS_SENSE_DIR_REVERSED_NAME = "bus_sense_dir_reversed";
    inline void clear_bus_sense_dir_reversed() { bus_sense_dir_reversed_.clear(); }
    inline void set_bus_sense_dir_reversed(const bool& value) { bus_sense_dir_reversed_ = value; }
    inline void set_bus_sense_dir_reversed(const bool&& value) { bus_sense_dir_reversed_ = value; }
    inline bool& mutable_bus_sense_dir_reversed() { return bus_sense_dir_reversed_.get(); }
    inline const bool& get_bus_sense_dir_reversed() const { return bus_sense_dir_reversed_.get(); }
    inline bool bus_sense_dir_reversed() const { return bus_sense_dir_reversed_.get(); }

    static constexpr char const* MAX_REGEN_CURRENT_NAME = "max_regen_current";
    inline void clear_max_regen_current() { max_regen_current_.clear(); }
    inline void set_max_regen_current(const float& value) { max_regen_current_ = value; }
    inline void set_max_regen_current(const float&& value) { max_regen_current_ = value; }
    inline float& mutable_max_regen_current() { return max_regen_current_.get(); }
    inline const float& get_max_regen_current() const { return max_regen_current_.get(); }
    inline float max_regen_current() const { return max_regen_current_.get(); }

    static constexpr char const* CURRENT_SENSE_GAIN_NAME = "current_sense_gain";
    inline void clear_current_sense_gain() { current_sense_gain_.clear(); }
    inline void set_current_sense_gain(const float& value) { current_sense_gain_ = value; }
    inline void set_current_sense_gain(const float&& value) { current_sense_gain_ = value; }
    inline float& mutable_current_sense_gain() { return current_sense_gain_.get(); }
    inline const float& get_current_sense_gain() const { return current_sense_gain_.get(); }
    inline float current_sense_gain() const { return current_sense_gain_.get(); }

    static constexpr char const* CURRENT_SENSE_SHUNT_OHM_NAME = "current_sense_shunt_ohm";
    inline void clear_current_sense_shunt_ohm() { current_sense_shunt_ohm_.clear(); }
    inline void set_current_sense_shunt_ohm(const float& value) { current_sense_shunt_ohm_ = value; }
    inline void set_current_sense_shunt_ohm(const float&& value) { current_sense_shunt_ohm_ = value; }
    inline float& mutable_current_sense_shunt_ohm() { return current_sense_shunt_ohm_.get(); }
    inline const float& get_current_sense_shunt_ohm() const { return current_sense_shunt_ohm_.get(); }
    inline float current_sense_shunt_ohm() const { return current_sense_shunt_ohm_.get(); }

    static constexpr char const* CURRENT_SENSE_DIR_REVERSED_NAME = "current_sense_dir_reversed";
    inline void clear_current_sense_dir_reversed() { current_sense_dir_reversed_.clear(); }
    inline void set_current_sense_dir_reversed(const bool& value) { current_sense_dir_reversed_ = value; }
    inline void set_current_sense_dir_reversed(const bool&& value) { current_sense_dir_reversed_ = value; }
    inline bool& mutable_current_sense_dir_reversed() { return current_sense_dir_reversed_.get(); }
    inline const bool& get_current_sense_dir_reversed() const { return current_sense_dir_reversed_.get(); }
    inline bool current_sense_dir_reversed() const { return current_sense_dir_reversed_.get(); }

    static constexpr char const* CURRENT_SENSE_F_LP_NAME = "current_sense_f_lp";
    inline void clear_current_sense_f_lp() { current_sense_f_lp_.clear(); }
    inline void set_current_sense_f_lp(const uint32_t& value) { current_sense_f_lp_ = value; }
    inline void set_current_sense_f_lp(const uint32_t&& value) { current_sense_f_lp_ = value; }
    inline uint32_t& mutable_current_sense_f_lp() { return current_sense_f_lp_.get(); }
    inline const uint32_t& get_current_sense_f_lp() const { return current_sense_f_lp_.get(); }
    inline uint32_t current_sense_f_lp() const { return current_sense_f_lp_.get(); }

    static constexpr char const* PLAY_STARTUP_TONE_NAME = "play_startup_tone";
    inline void clear_play_startup_tone() { play_startup_tone_.clear(); }
    inline void set_play_startup_tone(const bool& value) { play_startup_tone_ = value; }
    inline void set_play_startup_tone(const bool&& value) { play_startup_tone_ = value; }
    inline bool& mutable_play_startup_tone() { return play_startup_tone_.get(); }
    inline const bool& get_play_startup_tone() const { return play_startup_tone_.get(); }
    inline bool play_startup_tone() const { return play_startup_tone_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if((static_cast<UARTProtocol>(0) != uart_1_protocol_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = uart_1_protocol_.serialize_with_id(static_cast<uint32_t>(FieldNumber::UART_1_PROTOCOL), buffer, false);
      }

      if((static_cast<UARTBaudrate>(0) != uart_1_baudrate_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = uart_1_baudrate_.serialize_with_id(static_cast<uint32_t>(FieldNumber::UART_1_BAUDRATE), buffer, false);
      }

      if((0U != pwm_wave_freq_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = pwm_wave_freq_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PWM_WAVE_FREQ), buffer, false);
      }

      if((0U != speed_loop_freq_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = speed_loop_freq_.serialize_with_id(static_cast<uint32_t>(FieldNumber::SPEED_LOOP_FREQ), buffer, false);
      }

      if((0.0 != bus_overvoltage_limit_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_overvoltage_limit_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_OVERVOLTAGE_LIMIT), buffer, false);
      }

      if((0.0 != bus_undervoltage_limit_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_undervoltage_limit_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_UNDERVOLTAGE_LIMIT), buffer, false);
      }

      if((0.0 != bus_max_positive_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_max_positive_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_MAX_POSITIVE_CURRENT), buffer, false);
      }

      if((0.0 != bus_max_negative_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_max_negative_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_MAX_NEGATIVE_CURRENT), buffer, false);
      }

      if((0.0 != bus_sense_shunt_ohm_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_sense_shunt_ohm_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_SENSE_SHUNT_OHM), buffer, false);
      }

      if((false != bus_sense_dir_reversed_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = bus_sense_dir_reversed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUS_SENSE_DIR_REVERSED), buffer, false);
      }

      if((0.0 != max_regen_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_regen_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_REGEN_CURRENT), buffer, false);
      }

      if((0.0 != current_sense_gain_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = current_sense_gain_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURRENT_SENSE_GAIN), buffer, false);
      }

      if((0.0 != current_sense_shunt_ohm_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = current_sense_shunt_ohm_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURRENT_SENSE_SHUNT_OHM), buffer, false);
      }

      if((false != current_sense_dir_reversed_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = current_sense_dir_reversed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURRENT_SENSE_DIR_REVERSED), buffer, false);
      }

      if((0U != current_sense_f_lp_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = current_sense_f_lp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURRENT_SENSE_F_LP), buffer, false);
      }

      if((false != play_startup_tone_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = play_startup_tone_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PLAY_STARTUP_TONE), buffer, false);
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
          case FieldNumber::UART_1_PROTOCOL:
            return_value = uart_1_protocol_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::UART_1_BAUDRATE:
            return_value = uart_1_baudrate_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PWM_WAVE_FREQ:
            return_value = pwm_wave_freq_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::SPEED_LOOP_FREQ:
            return_value = speed_loop_freq_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_OVERVOLTAGE_LIMIT:
            return_value = bus_overvoltage_limit_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_UNDERVOLTAGE_LIMIT:
            return_value = bus_undervoltage_limit_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_MAX_POSITIVE_CURRENT:
            return_value = bus_max_positive_current_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_MAX_NEGATIVE_CURRENT:
            return_value = bus_max_negative_current_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_SENSE_SHUNT_OHM:
            return_value = bus_sense_shunt_ohm_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BUS_SENSE_DIR_REVERSED:
            return_value = bus_sense_dir_reversed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_REGEN_CURRENT:
            return_value = max_regen_current_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURRENT_SENSE_GAIN:
            return_value = current_sense_gain_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURRENT_SENSE_SHUNT_OHM:
            return_value = current_sense_shunt_ohm_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURRENT_SENSE_DIR_REVERSED:
            return_value = current_sense_dir_reversed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURRENT_SENSE_F_LP:
            return_value = current_sense_f_lp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PLAY_STARTUP_TONE:
            return_value = play_startup_tone_.deserialize_check_type(buffer, wire_type);
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
      clear_uart_1_protocol();
      clear_uart_1_baudrate();
      clear_pwm_wave_freq();
      clear_speed_loop_freq();
      clear_bus_overvoltage_limit();
      clear_bus_undervoltage_limit();
      clear_bus_max_positive_current();
      clear_bus_max_negative_current();
      clear_bus_sense_shunt_ohm();
      clear_bus_sense_dir_reversed();
      clear_max_regen_current();
      clear_current_sense_gain();
      clear_current_sense_shunt_ohm();
      clear_current_sense_dir_reversed();
      clear_current_sense_f_lp();
      clear_play_startup_tone();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::UART_1_PROTOCOL:
          name = UART_1_PROTOCOL_NAME;
          break;
        case FieldNumber::UART_1_BAUDRATE:
          name = UART_1_BAUDRATE_NAME;
          break;
        case FieldNumber::PWM_WAVE_FREQ:
          name = PWM_WAVE_FREQ_NAME;
          break;
        case FieldNumber::SPEED_LOOP_FREQ:
          name = SPEED_LOOP_FREQ_NAME;
          break;
        case FieldNumber::BUS_OVERVOLTAGE_LIMIT:
          name = BUS_OVERVOLTAGE_LIMIT_NAME;
          break;
        case FieldNumber::BUS_UNDERVOLTAGE_LIMIT:
          name = BUS_UNDERVOLTAGE_LIMIT_NAME;
          break;
        case FieldNumber::BUS_MAX_POSITIVE_CURRENT:
          name = BUS_MAX_POSITIVE_CURRENT_NAME;
          break;
        case FieldNumber::BUS_MAX_NEGATIVE_CURRENT:
          name = BUS_MAX_NEGATIVE_CURRENT_NAME;
          break;
        case FieldNumber::BUS_SENSE_SHUNT_OHM:
          name = BUS_SENSE_SHUNT_OHM_NAME;
          break;
        case FieldNumber::BUS_SENSE_DIR_REVERSED:
          name = BUS_SENSE_DIR_REVERSED_NAME;
          break;
        case FieldNumber::MAX_REGEN_CURRENT:
          name = MAX_REGEN_CURRENT_NAME;
          break;
        case FieldNumber::CURRENT_SENSE_GAIN:
          name = CURRENT_SENSE_GAIN_NAME;
          break;
        case FieldNumber::CURRENT_SENSE_SHUNT_OHM:
          name = CURRENT_SENSE_SHUNT_OHM_NAME;
          break;
        case FieldNumber::CURRENT_SENSE_DIR_REVERSED:
          name = CURRENT_SENSE_DIR_REVERSED_NAME;
          break;
        case FieldNumber::CURRENT_SENSE_F_LP:
          name = CURRENT_SENSE_F_LP_NAME;
          break;
        case FieldNumber::PLAY_STARTUP_TONE:
          name = PLAY_STARTUP_TONE_NAME;
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

      left_chars = uart_1_protocol_.to_string(left_chars, indent_level + 2, UART_1_PROTOCOL_NAME, true);
      left_chars = uart_1_baudrate_.to_string(left_chars, indent_level + 2, UART_1_BAUDRATE_NAME, false);
      left_chars = pwm_wave_freq_.to_string(left_chars, indent_level + 2, PWM_WAVE_FREQ_NAME, false);
      left_chars = speed_loop_freq_.to_string(left_chars, indent_level + 2, SPEED_LOOP_FREQ_NAME, false);
      left_chars = bus_overvoltage_limit_.to_string(left_chars, indent_level + 2, BUS_OVERVOLTAGE_LIMIT_NAME, false);
      left_chars = bus_undervoltage_limit_.to_string(left_chars, indent_level + 2, BUS_UNDERVOLTAGE_LIMIT_NAME, false);
      left_chars = bus_max_positive_current_.to_string(left_chars, indent_level + 2, BUS_MAX_POSITIVE_CURRENT_NAME, false);
      left_chars = bus_max_negative_current_.to_string(left_chars, indent_level + 2, BUS_MAX_NEGATIVE_CURRENT_NAME, false);
      left_chars = bus_sense_shunt_ohm_.to_string(left_chars, indent_level + 2, BUS_SENSE_SHUNT_OHM_NAME, false);
      left_chars = bus_sense_dir_reversed_.to_string(left_chars, indent_level + 2, BUS_SENSE_DIR_REVERSED_NAME, false);
      left_chars = max_regen_current_.to_string(left_chars, indent_level + 2, MAX_REGEN_CURRENT_NAME, false);
      left_chars = current_sense_gain_.to_string(left_chars, indent_level + 2, CURRENT_SENSE_GAIN_NAME, false);
      left_chars = current_sense_shunt_ohm_.to_string(left_chars, indent_level + 2, CURRENT_SENSE_SHUNT_OHM_NAME, false);
      left_chars = current_sense_dir_reversed_.to_string(left_chars, indent_level + 2, CURRENT_SENSE_DIR_REVERSED_NAME, false);
      left_chars = current_sense_f_lp_.to_string(left_chars, indent_level + 2, CURRENT_SENSE_F_LP_NAME, false);
      left_chars = play_startup_tone_.to_string(left_chars, indent_level + 2, PLAY_STARTUP_TONE_NAME, false);
  
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


      EmbeddedProto::enumeration<UARTProtocol> uart_1_protocol_ = static_cast<UARTProtocol>(0);
      EmbeddedProto::enumeration<UARTBaudrate> uart_1_baudrate_ = static_cast<UARTBaudrate>(0);
      EmbeddedProto::uint32 pwm_wave_freq_ = 0U;
      EmbeddedProto::uint32 speed_loop_freq_ = 0U;
      EmbeddedProto::floatfixed bus_overvoltage_limit_ = 0.0;
      EmbeddedProto::floatfixed bus_undervoltage_limit_ = 0.0;
      EmbeddedProto::floatfixed bus_max_positive_current_ = 0.0;
      EmbeddedProto::floatfixed bus_max_negative_current_ = 0.0;
      EmbeddedProto::floatfixed bus_sense_shunt_ohm_ = 0.0;
      EmbeddedProto::boolean bus_sense_dir_reversed_ = false;
      EmbeddedProto::floatfixed max_regen_current_ = 0.0;
      EmbeddedProto::floatfixed current_sense_gain_ = 0.0;
      EmbeddedProto::floatfixed current_sense_shunt_ohm_ = 0.0;
      EmbeddedProto::boolean current_sense_dir_reversed_ = false;
      EmbeddedProto::uint32 current_sense_f_lp_ = 0U;
      EmbeddedProto::boolean play_startup_tone_ = false;

};

} // End of namespace Config
} // End of namespace DataType
} // End of namespace iFOC
#endif // CONFIG_BOARD_BOARD_CONFIG_H