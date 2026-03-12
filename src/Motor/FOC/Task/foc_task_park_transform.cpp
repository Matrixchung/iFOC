#include "foc_task_park_transform.hpp"

static constexpr uint8_t OVERCURRENT_DETECT_TICKS = 5;

namespace iFOC
{
ParkTransformTask::ParkTransformTask() : Task("Park")
{
    RegisterTask(TaskType::RT_TASK);
}

void ParkTransformTask::UpdateRT(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    foc->Iqd_measured = FOC_Park(foc->Ialphabeta_measured, foc->elec_angle_rad);
    if(MAX(ABS(foc->Iqd_measured.q), ABS(foc->Iqd_measured.d)) >= foc->GetConfig().max_current() * 1.2f) // 120% max current, no UVLO
    {
        overcurrent_tick++;
        if(overcurrent_tick >= OVERCURRENT_DETECT_TICKS &&
            !foc->CheckError(MotorError::MOTOR_DC_BUS_UNDERVOLTAGE))
        {
            foc->DisarmWithError(MotorError::MOTOR_PHASE_D_Q_AXIS_OVER_CURRENT);
        }
    }
    else if(!foc->CheckError(MotorError::MOTOR_PHASE_D_Q_AXIS_OVER_CURRENT))
    {
        overcurrent_tick = 0;
    }
}
}