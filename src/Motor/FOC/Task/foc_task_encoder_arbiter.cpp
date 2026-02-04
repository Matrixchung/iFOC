#include "foc_task_encoder_arbiter.hpp"

// #define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
EncoderArbiterTask::EncoderArbiterTask() : Task("EncArbiter")
{
    RegisterTask(TaskType::RT_TASK);
}

void EncoderArbiterTask::UpdateRT(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    if(const auto enc = foc->GetPrimaryEncoder())
    {
        if(!enc->IsResultValid())
        {
            if(foc->state_machine.GetState() == MotorState::SENSORED_CLOSED_LOOP_CONTROL)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_RESULT_INVALID);
            }
            foc->elec_angle_rad = 0.0f;
            foc->elec_omega_rad_s = 0.0f;
            return;
        }
        if(foc->state_machine.GetState() != MotorState::ENCODER_CALIBRATION)
        {
            enc->SetSpeedFilterFreq(foc->GetConfig().sensor_speed_f_lp());
            foc->elec_angle_rad = enc->single_round_angle_rad;
            foc->elec_omega_rad_s = enc->angular_speed_rad_s;
            if(enc->GetEncoderType() != Encoder::Type::SENSORLESS_ENCODER &&
               foc->GetConfig().pole_pairs_valid())
            {
                if(foc->GetConfig().sensor_zero_offset_valid())
                    foc->elec_angle_rad = normalize_rad(foc->elec_angle_rad * foc->GetConfig().pole_pairs() - foc->GetConfig().sensor_zero_offset_rad());
                else foc->elec_angle_rad = normalize_rad(foc->elec_angle_rad * foc->GetConfig().pole_pairs());

                // enc->SetSpeedFilterFreq(foc->GetConfig().sensor_speed_f_lp());
                foc->elec_omega_rad_s *= foc->GetConfig().pole_pairs();
            }
        }
    }
    else
    {
        if(foc->state_machine.GetState() == MotorState::SENSORED_CLOSED_LOOP_CONTROL)
        {
            foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
        }
        foc->elec_angle_rad = 0.0f;
        foc->elec_omega_rad_s = 0.0f;
    }
}
}