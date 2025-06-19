#pragma once

#include "foc_speed_loop_base.hpp"
#include "pi_controller.hpp"

namespace iFOC::FOC
{
class SpeedLoopPI final : public SpeedLoopBase
{
public:
    SpeedLoopPI();
    PIController speed_pi{};
    PIController pos_pi{};
    void InitSpeedLoop() final;
    void UpdateSpeedLoop(float Ts) final;
    void ResetSpeedLoop() final;
private:
    Motion current{};
    Motion target{};
};
}