#pragma once

#include "../foc_motor.hpp"

namespace iFOC
{
class UpdateSenseTask final : public Task
{
public:
    UpdateSenseTask();
    void UpdateRT(float Ts) override;
    void UpdateNormal() override;
private:
    uint8_t overcurrent_tick = 0;
    uint8_t calibration_timeout_ms = 0;
    uint8_t temperature_sense_tick = 0;
};
}