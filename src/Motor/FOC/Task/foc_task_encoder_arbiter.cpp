#include "foc_task_encoder_arbiter.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
EncoderArbiterTask::EncoderArbiterTask() : Task("EncArbiter")
{
    RegisterTask(TaskType::RT_TASK);
}

void EncoderArbiterTask::UpdateRT(float Ts)
{
    if(auto enc = foc->GetPrimaryEncoder())
    {
        if(!enc.value()->IsResultValid())
        {
            foc->elec_angle_rad = 0.0f;
            foc->elec_omega_rad_s = 0.0f;
            return;
        }
        foc->elec_angle_rad = enc.value()->single_round_angle_rad;
        foc->elec_omega_rad_s = enc.value()->angular_speed_rad_s;
        // foc->mech_speed_rpm = RAD2RPM(enc.value()->angular_speed_rad_s, 1); // here the pole_pair = 1
        if(enc.value()->GetEncoderType() != Encoder::Type::SENSORLESS_ENCODER &&
           foc->GetConfig().pole_pairs_valid())
        {
            if(foc->GetConfig().sensor_zero_offset_valid())
                foc->elec_angle_rad = normalize_rad(normalize_rad(foc->elec_angle_rad * foc->GetConfig().pole_pairs()) - foc->GetConfig().sensor_zero_offset_rad());
            else foc->elec_angle_rad = normalize_rad(foc->elec_angle_rad * foc->GetConfig().pole_pairs());

            foc->elec_omega_rad_s *= foc->GetConfig().pole_pairs();
        }
    }
    else
    {
        foc->elec_angle_rad = 0.0f;
        foc->elec_omega_rad_s = 0.0f;
    }
}
}