#pragma once

#include "../foc_motor.hpp"
#include "foc_types.hpp"

namespace iFOC::FOC
{
/// Goal: calculate sensor direction, pole pairs(if flux linkage valid, we can also have torque_constant and KV), and zero offset to config.
class EncoderCalibTask final : public Task
{
private:
    enum class EstStage : uint8_t
    {
        NONE = 0,
        SENSOR_DIRECTION_TESTING,
        POLE_PAIRS_TESTING,
        SENSOR_ZERO_OFFSET_TESTING,
        SENSOR_CUSTOM_CALIBRATING,
    };
    EstStage stage = EstStage::NONE;
    uint8_t stage_passed = 0;
    bool is_sensor_custom_calibrated = false;
public:
    EncoderCalibTask();
    void InitNormal();
    void UpdateNormal() final;
};
}