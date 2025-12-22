#pragma once

#include "../dc_motor.hpp"
#include "pi_controller.hpp"

namespace iFOC::DC
{
class SpeedLoopPI final : public Task
{
public:
    SpeedLoopPI();
    PIController speed_pi{};
    PIController pos_pi{};
    void InitMid() override;
    void UpdateMid(float Ts) override;
    void ResetSpeedLoop();
private:
    Motion current{};
    Motion target{};
};
}
