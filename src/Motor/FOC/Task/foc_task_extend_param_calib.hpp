#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
class ExtendParamCalibTask final : public Task
{
    enum class EstStage : uint8_t
    {
        NONE = 0,
        ENCODER_OFF_AXIS_PEAK_FINDING = 1,
        ENCODER_OFF_AXIS_DIRECTION_TESTING = 2,
        ENCODER_OFF_AXIS_LUT_CALIBRATING = 3,
    };
    EstStage stage = EstStage::NONE;
    uint8_t stage_passed = 0;
public:
    ExtendParamCalibTask();
    void InitNormal() override;
    void UpdateNormal() override;
};
}