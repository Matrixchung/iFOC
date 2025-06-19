#include "foc_task_update_sense.hpp"

#define foc GetMotor<FOCMotor>()
static constexpr uint8_t OVERCURRENT_DETECT_TICKS = 5;

namespace iFOC
{
UpdateSenseTask::UpdateSenseTask() : Task("SenseTask")
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 2;
}

void UpdateSenseTask::UpdateRT(float Ts)
{
    foc->GetCurrSense()->Update(Ts);
    foc->Ialphabeta_measured = FOC_Clark(foc->GetCurrSense()->shunt_values);
    foc->Iqd_measured = FOC_Park(foc->Ialphabeta_measured, foc->elec_angle_rad);
    if(MAX(ABS(foc->Iqd_measured.q), ABS(foc->Iqd_measured.d)) >= foc->GetConfig().max_current() * 1.1f) // 110% max current
    {
        overcurrent_tick++;
        if(overcurrent_tick > OVERCURRENT_DETECT_TICKS)
        {
            foc->DisarmWithError(MotorError::PHASE_D_Q_AXIS_OVER_CURRENT);
        }
    }
    else if(!foc->CheckError(MotorError::PHASE_D_Q_AXIS_OVER_CURRENT))
    {
        overcurrent_tick = 0;
    }
}

void UpdateSenseTask::UpdateNormal()
{
    if(foc->GetInternalID() == 0) foc->GetBusSense()->Update(); // only update BusSense if is primary instance
    if(foc->GetBusSense()->current > BoardConfig.GetConfig().bus_max_positive_current())
        foc->DisarmWithError(DataType::Base::MotorError::DC_BUS_OVER_DRAIN_CURRENT);
    if(foc->GetBusSense()->current < BoardConfig.GetConfig().bus_max_negative_current())
        foc->DisarmWithError(DataType::Base::MotorError::DC_BUS_OVER_RECHARGE_CURRENT);
    sleep(10);
}
}