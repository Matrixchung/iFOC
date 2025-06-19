#include "foc_curr_loop_base.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
CurrLoopBase::CurrLoopBase() : Task("CurrLoop")
{
    RegisterTask(TaskType::RT_TASK);
}

void CurrLoopBase::InitRT()
{
    InitCurrLoop();
}

void CurrLoopBase::UpdateRT(float Ts)
{
    if(foc->IsArmed()) UpdateCurrLoop(Ts);
    else ResetCurrLoop();
}

void CurrLoopBase::InitCurrLoop() {}

}