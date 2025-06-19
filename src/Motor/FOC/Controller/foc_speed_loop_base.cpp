#include "foc_speed_loop_base.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
SpeedLoopBase::SpeedLoopBase() : Task("SpeedLoop")
{
    RegisterTask(TaskType::MID_TASK);
}

void SpeedLoopBase::InitMid()
{
    InitSpeedLoop();
}

void SpeedLoopBase::UpdateMid(float Ts)
{
    if(foc->IsArmed())
    {
        UpdateSpeedLoop(Ts);
    }
    else ResetSpeedLoop();
}

void SpeedLoopBase::InitSpeedLoop() {}

}