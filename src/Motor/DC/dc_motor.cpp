#include "dc_motor.hpp"

#include "Controller/dc_speed_loop_pi.hpp"
#include "Task/dc_task_update.hpp"

using namespace iFOC::DC;

namespace iFOC
{
FuncRetCode DCMotor::Init(const bool initTIM)
{
    auto encoder = GetPrimaryEncoder();
    if(!driver)
    {
        ThrowError(MotorError::DRIVER_COMPONENT_MISSING);
        return FuncRetCode::INVALID_INPUT;
    }
    if(!bus_sense)
    {
        ThrowError(MotorError::BUS_SENSE_COMPONENT_MISSING);
        return FuncRetCode::INVALID_INPUT;
    }
    if(!encoder)
    {
        ThrowError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
        return FuncRetCode::INVALID_INPUT;
    }
    const auto& board = BoardConfig().GetConfig();
    if (board.get_bus_overvoltage_limit() <= 1.0f ||
        board.get_bus_undervoltage_limit() >= board.get_bus_overvoltage_limit() ||
        board.get_bus_max_positive_current() <= 0.01f ||
        board.get_bus_max_negative_current() >= -0.01f)
    {
        ThrowError(MotorError::CONFIG_BOARD_CONFIG_INVALID);
        return FuncRetCode::PARAM_OUT_BOUND;
    }
    if(config.ReadNVMConfig() != FuncRetCode::OK) ThrowError(MotorError::SYSTEM_MOTOR_CONFIG_READ_ERROR);
    if(GetConfig().deduction_ratio() <= 0.0f) GetConfig().set_deduction_ratio(1.0f);
    auto result = bus_sense->Init();
    if(result != FuncRetCode::OK)
    {
        ThrowError(MotorError::BUS_SENSE_INIT_FAILED);
        return result;
    }
    bus_sense->Update();
    result = driver->Init(initTIM);
    if(result != FuncRetCode::OK)
    {
        driver->DisableAllOutputs();
        ThrowError(MotorError::DRIVER_INIT_FAILED);
        return result;
    }
    AppendTask(new DC::UpdateTask);
    AppendTask(new DC::SpeedLoopPI);
    Arm();
    return FuncRetCode::OK;
}

bool DCMotor::Arm()
{
    if(const auto curr = GetSpeedLoop()) curr.value()->ResetSpeedLoop();
    if(error == to_underlying(MotorError::NONE))
    {
        is_armed = true;
        GetDriver()->SetOutputPu(0.0f);
    }
    else Disarm();
    return error == to_underlying(MotorError::NONE);
}

void DCMotor::Disarm()
{
    is_armed = false;
    GetDriver()->SetOutputPu(0.0f);
}

void DCMotor::DisarmWithError(MotorError e)
{
    Disarm();
    // SetControlMode()
    ThrowError(e);
}

void DCMotor::GetCurrentMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit,
    Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit)
{
    dest.Reset();
    switch(ref_frame)
    {
        case Motion::Ref::ELEC:
        case Motion::Ref::BASE:
        {
            dest.torque = {Idc_measured, Motion::TorqueUnit::AMP};
            if(const auto& enc = GetPrimaryEncoder())
            {
                dest.speed = {enc.value()->angular_speed_rad_s, Motion::SpeedUnit::RADS};
                dest.pos = {enc.value()->multi_round_angle_rad, Motion::PosUnit::RAD};
            }
            break;
        }
        case Motion::Ref::OUTPUT:
        {
            dest.torque = {Idc_measured, Motion::TorqueUnit::AMP};
            if(const auto& enc = GetPrimaryEncoder())
            {
                const real_t temp = 1.0f / GetConfig().deduction_ratio();
                dest.speed = {enc.value()->angular_speed_rad_s * temp, Motion::SpeedUnit::RADS};
                dest.pos = {enc.value()->multi_round_angle_rad * temp, Motion::PosUnit::RAD};
            }
        }
        default: break;
    }
    dest.ConvertSpeedPosFromDefault(speed_unit, pos_unit);
}

void DCMotor::GetTargetMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit,
    Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit)
{
    dest = current_target;
    if(dest.ref != Motion::Ref::BASE)
    {
        dest.Reset();
        return;
    }
    if(ref_frame == Motion::Ref::OUTPUT)
    {
        const real_t temp = 1.0f / GetConfig().deduction_ratio();
        dest.speed = {dest.speed.value * temp, dest.speed.limit * temp, Motion::SpeedUnit::RADS};
        dest.pos = {dest.pos.value * temp, dest.pos.limit * temp, Motion::PosUnit::RAD};
    }
    dest.ConvertSpeedPosFromDefault(speed_unit, pos_unit);
}

void DCMotor::SetTargetMotion(Motion& motion)
{
    if(motion.ref == Motion::Ref::ELEC || motion.torque.unit != Motion::TorqueUnit::AMP) return;
    motion.ConvertSpeedPosToDefault();
    if(motion.ref == Motion::Ref::OUTPUT)
    {
        if(GetConfig().deduction_ratio() <= 0.0f) return; // deduction ratio invalid, return
        motion.speed.value *= GetConfig().deduction_ratio(); // OUTPUT -> BASE
        motion.speed.limit *= GetConfig().deduction_ratio();
        motion.pos.value *= GetConfig().deduction_ratio();
        motion.pos.limit *= GetConfig().deduction_ratio();
    }
    motion.ref = Motion::Ref::BASE;
    current_target = motion;
}

std::optional<DC::SpeedLoopPI*> DCMotor::GetSpeedLoop()
{
    auto speedloop = GetTaskByName("SpeedLoopPI");
    if(speedloop) return std::make_optional(reinterpret_cast<SpeedLoopPI*>(speedloop.value()));
    return std::nullopt;
}
}
