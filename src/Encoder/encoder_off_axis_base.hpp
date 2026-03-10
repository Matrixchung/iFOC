#pragma once

#include "encoder_base.hpp"
#include "../DataType/lookup_table.hpp"

/*
 * Theory: SLYA036B - Linear Hall-Effect Sensor Angle Measurement Theory, Implementation, and Calibration, Texas Instruments 2021.
 *
 */

namespace iFOC::Encoder
{
constexpr char OFF_AXIS_PEAK_DB_KEY_PREFIX[] = "ofp";
constexpr char OFF_AXIS_LUT_DB_KEY_PREFIX[] = "ofl";
constexpr uint16_t OFF_AXIS_LUT_POINTS = 72; // 5° per sample

class EncoderOffAxisBase : public EncoderBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(EncoderOffAxisBase);
public:
    enum class CalibrationState : uint8_t
    {
        NONE = 0,
        PEAK = 1,
        LUT = 2
    };
    EncoderOffAxisBase();
    ~EncoderOffAxisBase() override = default;
    [[nodiscard]] virtual float GetChannelA_mV() = 0;
    [[nodiscard]] virtual float GetChannelB_mV() = 0;
    [[nodiscard]] virtual bool IsConnected() = 0;
    FuncRetCode Init(uint8_t motor_id) override;
    void UpdateMid(float Ts) override;
    void SaveConfig(uint8_t motor_id) override;
    void SetCalibrationMode(CalibrationState state);
    [[nodiscard]] std::pair<float, float> GetChannelA_MinMax_mV() const;
    [[nodiscard]] std::pair<float, float> GetChannelB_MinMax_mV() const;
    [[nodiscard]] float GetChannelA_Amplitude_mV() const;
    [[nodiscard]] float GetChannelB_Amplitude_mV() const;
    [[nodiscard]] float GetChannelA_Normed();
    [[nodiscard]] float GetChannelB_Normed();
    [[nodiscard]] float GetAtan2();
    [[nodiscard]] bool IsCalibrated() const;
    [[nodiscard]] bool IsPeakCalibrated() const;
    [[nodiscard]] bool IsLUTCalibrated() const;
    DataType::LookupTable offset_lut;
protected:
    union
    {
        uint8_t reg = 0;
        struct
        {
            uint8_t hw_ready           : 1; /* [0] */
            uint8_t mag_field_weak     : 1; /* [1] */
            uint8_t mag_field_overflow : 1; /* [2] */
            uint8_t                    : 5; /* [3:7] */
        } bit;
    } flags;
    CalibrationState calib_state = CalibrationState::NONE;
    struct param_t
    {
        float Vmin = std::numeric_limits<float>::max();
        float Vmax = std::numeric_limits<float>::lowest();
        float Vamp = 0.0f;
    };
    param_t param_ChA{};
    param_t param_ChB{};
    real_t last_compensated_angle_rad = 0.0f;
};
}