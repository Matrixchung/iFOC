#include "dc_task_update.hpp"

#define dc GetMotor<DCMotor>()

namespace iFOC::DC
{
UpdateTask::UpdateTask() : Task("DCUpdTask")
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 2;
}

void UpdateTask::UpdateRT(float Ts)
{
    auto curr_sense = dc->GetCurrSense();
    if(curr_sense)
    {
        curr_sense->Update(Ts);
    }
}

void UpdateTask::UpdateNormal()
{
    if(dc->GetInternalID() == 0) dc->GetBusSense()->Update();
    if(dc->GetBusSense()->current > BoardConfig().GetConfig().bus_max_positive_current())
        dc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_DRAIN_CURRENT);
    if(dc->GetBusSense()->current < BoardConfig().GetConfig().bus_max_negative_current())
        dc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_RECHARGE_CURRENT);
    if(const auto core = dc->GetCoreTempSense())
        core->Update();
    if(const auto mosfet = dc->GetMosfetTempSense())
        mosfet->Update();
    if(const auto motor = dc->GetMotorTempSense())
        motor->Update();
    sleep(10);
}
}
