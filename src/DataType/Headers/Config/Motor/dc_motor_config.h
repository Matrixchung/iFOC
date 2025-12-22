/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Config/Motor/dc_motor_config.proto
 */

// This file is generated. Please do not edit!
#ifndef CONFIG_MOTOR_DC_MOTOR_CONFIG_H
#define CONFIG_MOTOR_DC_MOTOR_CONFIG_H

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

class DCMotorConfig final: public ::EmbeddedProto::MessageInterface
{
  public:
        REFLECT(
        MEMBER_SIZE_OFFSET(DCMotorConfig, node_id_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, max_voltage_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, max_current_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, deduction_ratio_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, max_output_speed_rpm_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, pos_kp_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, vel_kp_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, vel_ki_),
        MEMBER_SIZE_OFFSET(DCMotorConfig, watchdog_timeout_sec_)
    )
DCMotorConfig() = default;
    DCMotorConfig(const DCMotorConfig& rhs )
    {
      set_node_id(rhs.get_node_id());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
    }

    DCMotorConfig(const DCMotorConfig&& rhs ) noexcept
    {
      set_node_id(rhs.get_node_id());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
    }

    ~DCMotorConfig() override = default;

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      NODE_ID = 1,
      MAX_VOLTAGE = 9,
      MAX_CURRENT = 10,
      DEDUCTION_RATIO = 29,
      MAX_OUTPUT_SPEED_RPM = 30,
      POS_KP = 31,
      VEL_KP = 32,
      VEL_KI = 33,
      WATCHDOG_TIMEOUT_SEC = 34
    };

    DCMotorConfig& operator=(const DCMotorConfig& rhs)
    {
      set_node_id(rhs.get_node_id());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      return *this;
    }

    DCMotorConfig& operator=(const DCMotorConfig&& rhs) noexcept
    {
      set_node_id(rhs.get_node_id());
      set_max_voltage(rhs.get_max_voltage());
      set_max_current(rhs.get_max_current());
      set_deduction_ratio(rhs.get_deduction_ratio());
      set_max_output_speed_rpm(rhs.get_max_output_speed_rpm());
      set_pos_kp(rhs.get_pos_kp());
      set_vel_kp(rhs.get_vel_kp());
      set_vel_ki(rhs.get_vel_ki());
      set_watchdog_timeout_sec(rhs.get_watchdog_timeout_sec());
      return *this;
    }

    static constexpr char const* NODE_ID_NAME = "node_id";
    inline void clear_node_id() { node_id_.clear(); }
    inline void set_node_id(const uint32_t& value) { node_id_ = value; }
    inline void set_node_id(const uint32_t&& value) { node_id_ = value; }
    inline uint32_t& mutable_node_id() { return node_id_.get(); }
    inline const uint32_t& get_node_id() const { return node_id_.get(); }
    inline uint32_t node_id() const { return node_id_.get(); }

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


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if((0U != node_id_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = node_id_.serialize_with_id(static_cast<uint32_t>(FieldNumber::NODE_ID), buffer, false);
      }

      if((0.0 != max_voltage_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_voltage_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_VOLTAGE), buffer, false);
      }

      if((0.0 != max_current_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = max_current_.serialize_with_id(static_cast<uint32_t>(FieldNumber::MAX_CURRENT), buffer, false);
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
          case FieldNumber::NODE_ID:
            return_value = node_id_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_VOLTAGE:
            return_value = max_voltage_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::MAX_CURRENT:
            return_value = max_current_.deserialize_check_type(buffer, wire_type);
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
      clear_node_id();
      clear_max_voltage();
      clear_max_current();
      clear_deduction_ratio();
      clear_max_output_speed_rpm();
      clear_pos_kp();
      clear_vel_kp();
      clear_vel_ki();
      clear_watchdog_timeout_sec();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::NODE_ID:
          name = NODE_ID_NAME;
          break;
        case FieldNumber::MAX_VOLTAGE:
          name = MAX_VOLTAGE_NAME;
          break;
        case FieldNumber::MAX_CURRENT:
          name = MAX_CURRENT_NAME;
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

      left_chars = node_id_.to_string(left_chars, indent_level + 2, NODE_ID_NAME, true);
      left_chars = max_voltage_.to_string(left_chars, indent_level + 2, MAX_VOLTAGE_NAME, false);
      left_chars = max_current_.to_string(left_chars, indent_level + 2, MAX_CURRENT_NAME, false);
      left_chars = deduction_ratio_.to_string(left_chars, indent_level + 2, DEDUCTION_RATIO_NAME, false);
      left_chars = max_output_speed_rpm_.to_string(left_chars, indent_level + 2, MAX_OUTPUT_SPEED_RPM_NAME, false);
      left_chars = pos_kp_.to_string(left_chars, indent_level + 2, POS_KP_NAME, false);
      left_chars = vel_kp_.to_string(left_chars, indent_level + 2, VEL_KP_NAME, false);
      left_chars = vel_ki_.to_string(left_chars, indent_level + 2, VEL_KI_NAME, false);
      left_chars = watchdog_timeout_sec_.to_string(left_chars, indent_level + 2, WATCHDOG_TIMEOUT_SEC_NAME, false);
  
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


      EmbeddedProto::uint32 node_id_ = 0U;
      EmbeddedProto::floatfixed max_voltage_ = 0.0;
      EmbeddedProto::floatfixed max_current_ = 0.0;
      EmbeddedProto::floatfixed deduction_ratio_ = 0.0;
      EmbeddedProto::floatfixed max_output_speed_rpm_ = 0.0;
      EmbeddedProto::floatfixed pos_kp_ = 0.0;
      EmbeddedProto::floatfixed vel_kp_ = 0.0;
      EmbeddedProto::floatfixed vel_ki_ = 0.0;
      EmbeddedProto::floatfixed watchdog_timeout_sec_ = 0.0;

};

} // End of namespace Motor
} // End of namespace Config
} // End of namespace DataType
} // End of namespace iFOC
#endif // CONFIG_MOTOR_DC_MOTOR_CONFIG_H