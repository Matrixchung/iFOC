#include "dc_speed_loop_pi.hpp"

#define dc GetMotor<DCMotor>()

namespace iFOC::DC
{
SpeedLoopPI::SpeedLoopPI() : Task("SpeedLoop")
{
    RegisterTask(TaskType::MID_TASK);
}

void SpeedLoopPI::InitMid()
{
    speed_pi.Kp = dc->GetConfig().vel_kp();
    speed_pi.Ki = dc->GetConfig().vel_ki();
    speed_pi.limit = dc->GetConfig().max_voltage();
    pos_pi.Kp = dc->GetConfig().pos_kp();
    // OUTPUT RPM -> BASE RADS
    const real_t coeff = RPM2RAD(dc->GetConfig().deduction_ratio(), 1);
    pos_pi.limit = dc->GetConfig().max_output_speed_rpm() * coeff;
}

void SpeedLoopPI::UpdateMid(float Ts)
{
    dc->GetTargetMotion(target,
                    Motion::Ref::BASE,
                    Motion::TorqueUnit::AMP,
                    Motion::SpeedUnit::RADS,
                    Motion::PosUnit::RAD);
    dc->GetCurrentMotion(current,
                    Motion::Ref::BASE,
                    Motion::TorqueUnit::AMP,
                    Motion::SpeedUnit::RADS,
                    Motion::PosUnit::RAD);
    speed_pi.Kp = dc->GetConfig().vel_kp();
    speed_pi.Ki = dc->GetConfig().vel_ki();
    speed_pi.limit = dc->GetConfig().max_voltage();

    pos_pi.Kp = dc->GetConfig().pos_kp();
    // OUTPUT RPM -> BASE RADS
    const real_t coeff = RPM2RAD(dc->GetConfig().deduction_ratio(), 1);
    pos_pi.limit = dc->GetConfig().max_output_speed_rpm() * coeff;

    switch(dc->GetControlMode())
    {
        case MotorControlMode::CTRL_MODE_POSITION:
        {
            float pos_error = target.pos.value - current.pos.value;
            float speed_error = pos_pi.GetOutput(pos_error, Ts) - current.speed.value;
            dc->Udc_target = speed_pi.GetOutput(speed_error, Ts);
            break;
        }
        case MotorControlMode::CTRL_MODE_VELOCITY:
        {
            float speed_error = target.speed.value - current.speed.value;
            dc->Udc_target = speed_pi.GetOutput(speed_error, Ts);
            break;
        }
        default: ResetSpeedLoop(); break;
    }
    if(dc->IsArmed())
    {
        const auto Vbus = dc->GetBusSense()->voltage;
        if(dc->GetError() == to_underlying(MotorError::NONE) && Vbus > 0.0f)
        {
            dc->GetDriver()->SetOutputPu(dc->Udc_target / Vbus);
        }
        else dc->Disarm();
    }
}

void SpeedLoopPI::ResetSpeedLoop()
{
    speed_pi.Reset();
    pos_pi.Reset();
    dc->Idc_target = 0.0f;
    dc->Udc_target = 0.0f;
}
}
