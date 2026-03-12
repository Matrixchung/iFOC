#pragma once

#include "../foc_motor.hpp"

/// Reference: M. Piccoli, M. Yim, "Anticogging: Torque Ripple Suppression, Modeling, and Parameter Selection",
///            The International Journal of Robotics Research, 2016, Vol. 35(1-3):148-160
/// Reference: https://zhuanlan.zhihu.com/p/357325093

namespace iFOC::FOC
{
class ExtendParamCalibTask final : public Task
{
    float* anticogging_cw_map = nullptr;
    float* anticogging_ccw_map = nullptr;
    struct
    {
        int32_t index = 0;
        float stable_timer = 0.0f;
        float per_step_timer = 0.0f;
        float start_single_round_rad = 0.0f;
        float start_multi_round_rad = 0.0f;
        float iq_meas_acc = 0.0f;
        uint8_t iq_count = 0;
        bool prev_anticogging_enabled = false;
    } anticogging;
    enum class EstStage : uint8_t
    {
        NONE = 0,
        ENCODER_OFF_AXIS_PEAK_FINDING = 1,
        ENCODER_OFF_AXIS_DIRECTION_TESTING = 2,
        ENCODER_OFF_AXIS_LUT_CALIBRATING = 3,
        ANTICOGGING_INIT = 4,
        ANTICOGGING_TESTING_CW = 5,
        ANTICOGGING_TESTING_CCW = 6,
        ANTICOGGING_TESTED = 7
    };
    EstStage stage = EstStage::NONE;
    uint8_t stage_passed = 0;
public:
    ExtendParamCalibTask();
    ~ExtendParamCalibTask() override;
    void InitNormal() override;
    void UpdateNormal() override;
    void UpdateMid(float Ts) override;
};
}