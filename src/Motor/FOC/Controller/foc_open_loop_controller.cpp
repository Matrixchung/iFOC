#include "foc_open_loop_controller.hpp"

namespace iFOC::FOC
{
OpenLoopController::OpenLoopController() : Task("OpenLoop")
{
    RegisterTask(TaskType::MID_TASK);
    encoder = new EncoderOpenLoop();
}

OpenLoopController::~OpenLoopController()
{
    const auto foc = GetMotor<FOCMotor>();
    if(foc->RemoveEncoderByName("OLEnc") != FuncRetCode::OK)
    {
        if(encoder) vPortFree(encoder); // if the encoder wasn't actually appended, we should manually free
    }
}

void OpenLoopController::InitMid()
{
    const auto foc = GetMotor<FOCMotor>();
    if(encoder)
    {
        encoder->result_valid = true;
        foc->AppendEncoder(encoder);
        foc->SetPrimaryEncoderIndex(foc->GetEncoders().size() - 1); // set primary encoder to new encoder
    }
    else foc->RemoveTaskByName(GetName());
}

void OpenLoopController::UpdateMid(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    // if(foc->IsRampingMotion())
    // {
        switch(foc->GetControlMode())
        {
            case MotorControlMode::CTRL_MODE_CURRENT:
            case MotorControlMode::CTRL_MODE_VELOCITY:
            {
                Motion target{};
                foc->GetTargetMotion(target,
                        Motion::Ref::BASE,
                        Motion::TorqueUnit::AMP,
                        Motion::SpeedUnit::RADS,
                        Motion::PosUnit::RAD);
                if(foc->GetControlMode() == MotorControlMode::CTRL_MODE_CURRENT)
                {
                    // Target: torque, in openloop controller we also support current control
                    // however without proper electrical angle, the motor won't start spin
                    // so basically the torque command is only for testing the current loop, without spinning the motor.
                    if(foc->IsRampingMotion()) foc->Iqd_target.q = 0.0f; // error state.
                    else foc->Iqd_target.q = target.torque.value;
                    break;
                }
                const Motion ramp_start = foc->GetRampedStartMotion();
                const Motion ramp_diff = foc->GetRampedDiffMotion();
                const float target_torque = ramp_start.torque.value + ramp_diff.torque.value;
                target.torque.value = target_torque;
                // we have velocity as target.speed.value
                // imitating encoder
                encoder->angular_speed_rad_s = target.speed.value;
                const auto last_single_round_rad = encoder->compensated_single_round_angle_rad;
                encoder->compensated_single_round_angle_rad = normalize_rad(encoder->compensated_single_round_angle_rad + target.speed.value * Ts);
                encoder->raw_single_round_angle_rad = encoder->compensated_single_round_angle_rad;
                const auto delta = encoder->compensated_single_round_angle_rad - last_single_round_rad;
                if(delta > PI) encoder->full_rotations--;
                else if(delta < -PI) encoder->full_rotations++;
                encoder->multi_round_angle_rad = encoder->full_rotations * PI2 + encoder->compensated_single_round_angle_rad;

                foc->Iqd_target = {target.torque.value, 0.0f};
                break;
            }
            default:
            {
                encoder->angular_speed_rad_s = 0.0f;
                foc->Iqd_target = {0.0f, 0.0f};
                break;
            }
        }
    // }
}

OpenLoopController::EncoderOpenLoop::EncoderOpenLoop() : EncoderBase("OLEnc", Encoder::Type::ABSOLUTE_ENCODER, 1)
{
}
}
