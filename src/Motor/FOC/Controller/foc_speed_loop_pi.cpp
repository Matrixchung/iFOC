#include "foc_speed_loop_pi.hpp"

// #define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
SpeedLoopPI::SpeedLoopPI() : SpeedLoopBase() {}

void SpeedLoopPI::InitSpeedLoop()
{
    const auto foc = GetMotor<FOCMotor>();
    speed_pi.Kp = foc->GetConfig().vel_kp();
    speed_pi.Ki = foc->GetConfig().vel_ki();
    speed_pi.limit = foc->config_max_current;
    pos_pi.Kp = foc->GetConfig().pos_kp();
    pos_pi.limit = foc->config_max_base_speed_rad_s;
}

void SpeedLoopPI::UpdateSpeedLoop(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    Motion target, current;
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
    const auto control_mode = foc->GetControlMode();
    if(control_mode != MotorControlMode::CTRL_MODE_HYBRID)
    {
        if(target.torque.limit > 0.0f) speed_pi.limit = MIN(foc->config_max_current, target.torque.limit);
        else speed_pi.limit = foc->config_max_current;
        if(target.speed.limit > 0.0f) pos_pi.limit = MIN(foc->config_max_base_speed_rad_s, target.speed.limit);
        else pos_pi.limit = foc->config_max_base_speed_rad_s;
    }
    else // On MIT mode, pos_pi & speed_pi are unused, target.pos.limit are used as Kp, target.speed.limit is used as Kd
    {
        speed_pi.limit = foc->config_max_current;
        pos_pi.limit = foc->config_max_base_speed_rad_s;
    }
    switch(control_mode)
    {
        case MotorControlMode::CTRL_MODE_POSITION:
        {
            // Target: pos, Feedforward: speed, torque
            const float pos_error_base = target.pos.value - current.pos.value; // RAD
            const float target_speed_base = pos_pi.GetOutput(pos_error_base, Ts) + target.speed.value; // RAD/S, +ff
            const float speed_error = target_speed_base - current.speed.value; // RAD/S
            foc->Iqd_target.q = speed_pi.GetOutput(speed_error, Ts) + target.torque.value; // Amp, +ff
            break;
        }
        case MotorControlMode::CTRL_MODE_VELOCITY:
        {
            // Target: speed, Feedforward: torque
            const float speed_error = target.speed.value - current.speed.value; // RAD/S
            foc->Iqd_target.q = speed_pi.GetOutput(speed_error, Ts) + target.torque.value; // Amp, +ff
            break;
        }
        case MotorControlMode::CTRL_MODE_CURRENT:
        {
            // Target: torque
            foc->Iqd_target.q = target.torque.value;
            break;
        }
        case MotorControlMode::CTRL_MODE_HYBRID:
        {
            // MIT control
            // #1: Parameter validity check is done by setting target, here we just check torque constant
            if(foc->GetConfig().torque_constant() <= 0.0f || !foc->GetConfig().torque_constant_valid())
            {
                foc->Iqd_target = {0.0f, 0.0f};
                break;
            }
            const float pos_error_base = target.pos.value - current.pos.value; // RAD
            const float speed_error_base = target.speed.value - current.speed.value; // RADS
            const float pos_term_Nm = pos_error_base * target.pos.limit; // multiply Kp
            const float speed_term_Nm = speed_error_base * target.speed.limit; // multiply Kd
            const float sum_term_amp = (pos_term_Nm + speed_term_Nm) / foc->GetConfig().torque_constant();
            const float target_iq_amp = _constrain((sum_term_amp + target.torque.value), -target.torque.limit, target.torque.limit);
            foc->Iqd_target.q = target_iq_amp;
            foc->Iqd_target.d = 0.0f;
            break;
        }
        default: ResetSpeedLoop(); break;
    }
}

void SpeedLoopPI::ResetSpeedLoop()
{
    const auto foc = GetMotor<FOCMotor>();
    speed_pi.Reset();
    foc->Iqd_target = {0.0f, 0.0f};
}
}