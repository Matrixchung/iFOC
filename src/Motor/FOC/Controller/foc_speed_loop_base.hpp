#pragma once

#include "../foc_motor.hpp"

namespace iFOC::FOC
{
// SpeedLoop: Input <- [
//            Output -> Iqd_target
class SpeedLoopBase : public Task
{
public:
    SpeedLoopBase();
    void InitMid() final;
    void UpdateMid(float Ts) final;
    virtual void InitSpeedLoop();
    virtual void UpdateSpeedLoop(float Ts) = 0;
    virtual void ResetSpeedLoop() = 0;
};
}