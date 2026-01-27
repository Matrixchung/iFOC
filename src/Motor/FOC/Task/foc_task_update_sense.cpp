#include "foc_task_update_sense.hpp"


static constexpr uint8_t CALIBRATION_TIMEOUT_MS = 50;
static constexpr uint8_t TEMP_SENSE_UPDATE_TICKS = 5;

namespace iFOC
{
UpdateSenseTask::UpdateSenseTask() : Task("SenseTask")
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.stack_depth = 512;
    config.rtos_priority = configMAX_PRIORITIES - 3;
}

void UpdateSenseTask::UpdateRT(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    foc->GetCurrSense()->Update(Ts);
    // only enable leakage current detection after basic param calibration (Rs/Ld calibration will disconnect one of three phases)
    if(to_underlying(foc->state_machine.GetState()) > to_underlying(MotorState::BASIC_PARAM_CALIBRATION))
    {
        // Typically, we have Ia + Ib + Ic == 0. For three-shunt detection methods, leakage current can be detected.
        const real_t leakage_current = foc->GetCurrSense()->shunt_values[0] + foc->GetCurrSense()->shunt_values[1] + foc->GetCurrSense()->shunt_values[2];
        if(ABS(leakage_current) >= foc->GetConfig().max_current() * 0.2f) // max leakage current = 20% max current
        {
            foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE); // phase current imbalance
        }
    }
    foc->Ialphabeta_measured = FOC_Clark(foc->GetCurrSense()->shunt_values);
}

void UpdateSenseTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    auto* sense = foc->GetBusSense();
    if(foc->GetInternalID() == 0) // only update BusSense if is primary instance
    {
        const auto ret = sense->Update();
        if(ret != FuncRetCode::OK) foc->ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
        else foc->ClearError(MotorError::BUS_SENSE_RESULT_INVALID);
    }
    if(sense->voltage <= 0.0f)
    {
        foc->ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
    }
    else
    {
        foc->ClearError(MotorError::BUS_SENSE_RESULT_INVALID);
        if(sense->voltage > BoardConfig().GetConfig().bus_overvoltage_limit())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVERVOLTAGE);
        if(sense->voltage < BoardConfig().GetConfig().bus_undervoltage_limit())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_UNDERVOLTAGE);
        if(sense->current > BoardConfig().GetConfig().bus_max_positive_current())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_DRAIN_CURRENT);
        if(sense->current < BoardConfig().GetConfig().bus_max_negative_current())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_RECHARGE_CURRENT);
    }
    if(!foc->GetCurrSense()->IsCalibrated())
    {
        if(calibration_timeout_ms < CALIBRATION_TIMEOUT_MS) calibration_timeout_ms += 10;
        else foc->DisarmWithError(MotorError::MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT);
    }
    else
    {
        calibration_timeout_ms = 0;
        foc->ClearError(MotorError::MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT);
    }
    if(temperature_sense_tick++ >= TEMP_SENSE_UPDATE_TICKS)
    {
        if(const auto core = foc->GetCoreTempSense())
            core->Update();
        if(const auto mosfet = foc->GetMosfetTempSense())
            mosfet->Update();
        if(const auto motor = foc->GetMotorTempSense())
            motor->Update();
        temperature_sense_tick = 0;
    }
    if(const auto ind = foc->GetIndicator())
        ind->Update(foc->GetInternalID(), foc->GetError(), foc->GetCurrentState(), foc->GetControlMode());

    sleep(10);
}
}