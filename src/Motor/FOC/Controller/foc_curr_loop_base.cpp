#include "foc_curr_loop_base.hpp"

// #define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
CurrLoopBase::CurrLoopBase() : Task("CurrLoop")
{
    RegisterTask(TaskType::RT_TASK);
}
}