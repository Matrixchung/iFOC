#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
/// Goal: calculate sensor direction, pole pairs(if flux linkage valid, we can also have torque_constant and KV), and zero offset to config.
class EncoderCalibTask final : public Task
{
private:
    struct
    {
        int16_t sample_count = 0;
        float timer = 0.0f;
        float next_sample_time = 0.0f;
        float elec_angle_rad = 0.0f;
    } zero_est;
    float* temp_map = nullptr;
    enum class EstStage : uint8_t
    {
        NONE = 0,
        SENSOR_DIRECTION_TESTING,
        POLE_PAIRS_TESTING,
        SENSOR_ZERO_OFFSET_TEST_START,
        SENSOR_ZERO_OFFSET_TESTING_CW,
        SENSOR_ZERO_OFFSET_TESTING_CCW,
        SENSOR_ZERO_OFFSET_TESTED,
        // SENSOR_CUSTOM_CALIBRATING,
    };
    EstStage stage = EstStage::NONE;
    uint8_t stage_passed = 0;
    // bool is_sensor_custom_calibrated = false;
public:
    EncoderCalibTask();
    void InitNormal() override;
    void UpdateNormal() override;
    void UpdateMid(float Ts) override;
};
}