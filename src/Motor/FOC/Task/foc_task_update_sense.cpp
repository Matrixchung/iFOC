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
    // only enable leakage current detection after basic param calibration (Rs/Ld calibration will disconnect one of three phases)
    if(to_underlying(foc->state_machine.GetState()) > to_underlying(MotorState::BASIC_PARAM_CALIBRATION))
    {
        // Typically, we have Ia + Ib + Ic == 0. For three-shunt detection methods, leakage current can be detected.
        real_t leakage_current = foc->GetCurrSense()->shunt_values[0] + foc->GetCurrSense()->shunt_values[1] + foc->GetCurrSense()->shunt_values[2];
        if(ABS(leakage_current) >= foc->GetConfig().max_current() * 0.1f) // max leakage current = 10% max current
        {
            foc->DisarmWithError(MotorError::PHASE_IMBALANCE); // phase current imbalance
        }
    }
    foc->Ialphabeta_measured = FOC_Clark(foc->GetCurrSense()->shunt_values);
    foc->Iqd_measured = FOC_Park(foc->Ialphabeta_measured, foc->elec_angle_rad);
    if(MAX(ABS(foc->Iqd_measured.q), ABS(foc->Iqd_measured.d)) >= foc->GetConfig().max_current() * 1.2f) // 120% max current
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