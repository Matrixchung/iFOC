#pragma once

#include "foc_types.hpp"
#include "foc_math.hpp"
#include "../Common/Filter/lowpass_filter.hpp"
#include "../Common/foc_task.hpp"

namespace iFOC::Encoder
{
enum class Type : uint8_t
{
    ABSOLUTE_ENCODER = 0,
    INCREMENTAL_ENCODER = 1,
    SENSORLESS_ENCODER = 2
};
class EncoderBase
{
    OVERRIDE_NEW();
public:
    EncoderBase(const char *_name, Type _type, int8_t _s) :
    encoder_type(_type),
    sign_and_deduction_ratio(_s == -1 ? -1.0f : 1.0f)
    {
        strncpy(name, _name, sizeof(name));
    };
    virtual ~EncoderBase() = default;
    virtual FuncRetCode Init() = 0;
    virtual void UpdateRT(float Ts) {};
    virtual void UpdateMid(float Ts) {};
    // Since Normal task is running in RTOS loop and delay time is not proven,
    // all encoder instance shouldn't run a normal task.
    [[nodiscard]] const char* GetName() const { return name; };
    [[nodiscard]] bool IsResultValid() const { return result_valid; }
    [[nodiscard]] Type GetEncoderType() const { return encoder_type; }
    void SetSign(int8_t _sign)
    {
        sign_and_deduction_ratio = _sign >= 0 ? ABS(sign_and_deduction_ratio) : -ABS(sign_and_deduction_ratio);
    }
    [[nodiscard]] int8_t GetSign() const { return sign(sign_and_deduction_ratio); };
    /// For example, the deduction ratio from where the encoder mounts \n
    /// to end effector is 2:1 (encoder turns two rounds <=> end effector turns one round) \n
    /// Then the param ratio is 2.0
    /// \param ratio strictly >= 1.0, positive. Out-of-range input is ignored.
    void SetDeductionRatioToEndEffector(float ratio)
    {
        if(ratio < 1.0f) return;
        sign_and_deduction_ratio = sign(sign_and_deduction_ratio) * ratio;
    }
    void SetSpeedFilterFreq(real_t f) { speed_lpf.SetFc(f); };
    bool operator==(const char* n) const noexcept
    {
        return strncasecmp(GetName(), n, sizeof(name)) == 0;
    }
    bool operator!=(const char* n) const noexcept { return !(*this == n); };
    bool operator==(const EncoderBase& other) const noexcept { return (*this == other.GetName()); }
    bool operator!=(const EncoderBase& other) const noexcept { return !(*this == other); };
public:
    real_t single_round_angle_rad = 0.0f;
    real_t multi_round_angle_rad = 0.0f;
    real_t angular_speed_rad_s = 0.0f;
    long long full_rotations = 0;
protected:
    Type encoder_type;
    iFOC::Filter::LowpassFilter speed_lpf = iFOC::Filter::LowpassFilter(300);
    float sign_and_deduction_ratio = 1.0f;
    bool result_valid = false;
    char name[Task::MAX_TASK_NAME_LEN] = "\0";
};
}