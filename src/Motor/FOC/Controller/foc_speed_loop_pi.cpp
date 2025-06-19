#include "foc_speed_loop_pi.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
SpeedLoopPI::SpeedLoopPI() : SpeedLoopBase() {}

void SpeedLoopPI::InitSpeedLoop()
{
    speed_pi.Kp = foc->GetConfig().vel_kp();
    speed_pi.Ki = foc->GetConfig().vel_ki();
    speed_pi.limit = foc->config_max_current;
    pos_pi.Kp = foc->GetConfig().pos_kp();
    pos_pi.limit = foc->config_max_base_speed_rad_s;
}

void SpeedLoopPI::UpdateSpeedLoop(float Ts)
{
    foc->GetTargetMotion(target,
                    Motion::Ref::BASE,
                    Motion::TorqueUnit::AMP,
                    Motion::SpeedUnit::RADS,
                    Motion::PosUnit::RAD);
    foc->GetCurrentMotion(current,
                          Motion::Ref::BASE,
                          Motion::TorqueUnit::AMP,
                          Motion::SpeedUnit::RADS,
                          Motion::PosUnit::RAD);
    speed_pi.Kp = foc->GetConfig().vel_kp();
    speed_pi.Ki = foc->GetConfig().vel_ki();
    pos_pi.Kp = foc->GetConfig().pos_kp();
    if(target.torque.limit > 0.0f) speed_pi.limit = MIN(foc->config_max_current, target.torque.limit);
    else speed_pi.limit = foc->config_max_current;
    if(target.speed.limit > 0.0f) pos_pi.limit = MIN(foc->config_max_base_speed_rad_s, target.speed.limit);
    else pos_pi.limit = foc->config_max_base_speed_rad_s;
    switch(foc->GetControlMode())
    {
        case MotorControlMode::CTRL_MODE_POSITION:
        {
            // Target: pos, Feedforward: speed, torque
            float pos_error_base = target.pos.value - current.pos.value; // RAD
            float target_speed_base = pos_pi.GetOutput(pos_error_base, Ts) + target.speed.value; // RAD/S, +ff
            float speed_error = target_speed_base - current.speed.value; // RAD/S
            foc->Iqd_target.q = speed_pi.GetOutput(speed_error, Ts) + target.torque.value; // Amp, +ff
            break;
        }
        case MotorControlMode::CTRL_MODE_VELOCITY:
        {
            // Target: speed, Feedforward: torque
            float speed_error = target.speed.value - current.speed.value; // RAD/S
            foc->Iqd_target.q = speed_pi.GetOutput(speed_error, Ts) + target.torque.value; // Amp, +ff
            break;
        }
        case MotorControlMode::CTRL_MODE_CURRENT:
        {
            // Target: torque
            foc->Iqd_target.q = target.torque.value;
            break;
        }
        default: ResetSpeedLoop(); break;
    }
}

void SpeedLoopPI::ResetSpeedLoop()
{
    speed_pi.Reset();
    foc->Iqd_target = {0.0f, 0.0f};
}
}