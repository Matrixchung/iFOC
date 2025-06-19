#include "foc_motor.hpp"
#include "./Controller/foc_curr_loop_base.hpp"
#include "./Controller/foc_speed_loop_base.hpp"

#define DEFAULT_CURRENT_LOOP_BANDWIDTH (1000.0f)
#define DEFAULT_CALIBRATION_VOLTAGE (1.0f)
#define DEFAULT_CALIBRATION_CURRENT (1.0f)
#define DEFAULT_PARAM_MOTOR_MAX_VOLTAGE (12.0f)
#define DEFAULT_PARAM_MOTOR_MAX_CURRENT (10.0f)
#define DEFAULT_PARAM_MOTOR_MAX_OUTPUT_SPEED_RPM (1000.0f)

#define foc GetMotor<FOCMotor>()

using namespace iFOC::FOC;

namespace iFOC
{
FuncRetCode FOCMotor::Init(const bool initTIM)
{
    if(!driver || !curr_sense || !bus_sense)
    {
        misconfigured_area |= to_underlying(MisconfiguredArea::MOTOR_INIT_COMPONENTS_MISSING);
        return FuncRetCode::INVALID_INPUT;
    }
    // Config safety check, board config
    // BoardConfig.LinkNVMInterface(nvm); // the BoardConfig instance should be called earlier in the app_main(), but the fact is no.
    // BoardConfig.ReadNVMConfig();       // why?
    /// WARN Static global variable must be externed in header, instead of directly defined in header
    const auto& board = BoardConfig.GetConfig();
    if (board.get_bus_overvoltage_limit() <= 1.0f ||
        board.get_bus_undervoltage_limit() >= board.get_bus_overvoltage_limit() ||
        board.get_bus_max_positive_current() <= 0.01f ||
        board.get_bus_max_negative_current() >= -0.01f)
    {
        misconfigured_area |= to_underlying(MisconfiguredArea::MOTOR_INIT_BOARD_CONFIGS_INVALID);
        return FuncRetCode::PARAM_OUT_BOUND;
    }
    // Config safety check, motor config
    config.ReadNVMConfig();
    if (GetConfig().phase_inductance() <= 0.0f ||
        GetConfig().q_axis_inductance() <= 0.0f ||
        GetConfig().d_axis_inductance() <= 0.0f)
    {
        GetConfig().set_phase_inductance_valid(false);
    }
    if(GetConfig().pole_pairs() == 0) GetConfig().set_pole_pairs_valid(false);
    if(GetConfig().phase_resistance() <= 0.0001f) GetConfig().set_phase_resistance_valid(false);
    if(GetConfig().deduction_ratio() <= 0.0f) GetConfig().set_deduction_ratio(1.0f);
    if(GetConfig().max_output_speed_rpm() <= 0.0f) GetConfig().set_max_output_speed_rpm(DEFAULT_PARAM_MOTOR_MAX_OUTPUT_SPEED_RPM);
    if(GetConfig().watchdog_timeout_sec() <= 0.0f) GetConfig().set_watchdog_timeout_sec(0.0f);
    else
    {
        float temp = GetConfig().watchdog_timeout_sec() / iFOC::MID_LOOP_TS;
        if(temp >= 1.0f) watchdog_timeout_cnt = (uint32_t)temp;
    }
    auto result = bus_sense->Init();
    bus_sense->Update();
    if(result != FuncRetCode::OK)
    {
        misconfigured_area |= to_underlying(MisconfiguredArea::MOTOR_INIT_COMPONENTS_INIT_FAILED);
        return result;
    }
    if(GetConfig().current_loop_bandwidth() <= 0.0f ||
        GetConfig().calibration_voltage() <= 0.0f ||
        GetConfig().calibration_current() <= 0.0f ||
        GetConfig().max_voltage() <= 0.5f ||
        GetConfig().max_current() <= 0.0f)
    {
        ResetDefaultConfig();
    }
    config_max_current = GetConfig().max_current();
    config_max_base_speed_rad_s = RPM2RAD(GetConfig().max_output_speed_rpm() * GetConfig().deduction_ratio(), 1); // OUTPUT -> BASE frame
    result = driver->Init(initTIM);
    if(result != FuncRetCode::OK)
    {
        driver->DisableAllOutputs();
        misconfigured_area |= to_underlying(MisconfiguredArea::MOTOR_INIT_DRIVER_INIT_FAILED);
        return result;
    }
    if(GetConfig().sensor_direction_valid())
    {
        if(GetConfig().sensor_direction_clockwise() == false)
            // we need clockwise, so reverse primary encoder sign.
            if(auto enc = GetPrimaryEncoder()) enc.value()->SetSign(-1 * enc.value()->GetSign());
    }
    AppendTask(&this->state_machine);
    return FuncRetCode::OK;
}

FOCMotor::MotorBase::MotorError FOCMotor::Arm()
{
    if(const auto& curr = GetCurrLoop()) curr.value()->ResetCurrLoop();
    if(const auto& speed = GetSpeedLoop()) speed.value()->ResetSpeedLoop();
    if(error == MotorError::NONE)
    {
        is_armed = true;
        GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
        GetDriver()->EnableBridges(Driver::FOCDriverBase::Bridge::HB_U,
                                   Driver::FOCDriverBase::Bridge::LB_U,
                                   Driver::FOCDriverBase::Bridge::HB_V,
                                   Driver::FOCDriverBase::Bridge::LB_V,
                                   Driver::FOCDriverBase::Bridge::HB_W,
                                   Driver::FOCDriverBase::Bridge::LB_W);
    }
    else Disarm();
    return error;
}

void FOCMotor::Disarm()
{
    is_armed = false;
    GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
    GetDriver()->DisableBridges(Driver::FOCDriverBase::Bridge::HB_U,
                                Driver::FOCDriverBase::Bridge::LB_U,
                                Driver::FOCDriverBase::Bridge::HB_V,
                                Driver::FOCDriverBase::Bridge::LB_V,
                                Driver::FOCDriverBase::Bridge::HB_W,
                                Driver::FOCDriverBase::Bridge::LB_W);
}

void FOCMotor::DisarmWithError(MotorBase::MotorError e)
{
    Disarm();
    state_machine.RequestState(MotorState::IDLE);
    error = e;
}

void FOCMotor::GetCurrentMotion(Motion& ret, Motion::Ref r, Motion::TorqueUnit t, Motion::SpeedUnit s, Motion::PosUnit p)
{
    ret.Reset();
    real_t curr_torque_limit_base_amp = config_max_current;
    if(current_target.torque.limit > 0.0f) curr_torque_limit_base_amp = MIN(config_max_current, current_target.torque.limit);
    real_t curr_speed_limit_base_rad_s = config_max_base_speed_rad_s;
    if(current_target.speed.limit > 0.0f) curr_speed_limit_base_rad_s = MIN(config_max_base_speed_rad_s, current_target.speed.limit);
    switch(r)
    {
        case Motion::Ref::ELEC:
        {
            if(t == Motion::TorqueUnit::NM && GetConfig().torque_constant_valid())
            {
                ret.torque = {Iqd_measured.q * GetConfig().torque_constant(), curr_torque_limit_base_amp * GetConfig().torque_constant(), Motion::TorqueUnit::NM};
            }
            else ret.torque = {Iqd_measured.q, curr_torque_limit_base_amp, Motion::TorqueUnit::AMP};
            ret.speed = {elec_omega_rad_s, Motion::SpeedUnit::RADS};
            if(GetConfig().pole_pairs_valid()) ret.speed.limit = curr_speed_limit_base_rad_s * GetConfig().pole_pairs();
            ret.pos = {elec_angle_rad, Motion::PosUnit::RAD};
            break;
        }
        case Motion::Ref::BASE:
        {
            if(t == Motion::TorqueUnit::NM && GetConfig().torque_constant_valid())
            {
                ret.torque = {Iqd_measured.q * GetConfig().torque_constant(), curr_torque_limit_base_amp * GetConfig().torque_constant(), Motion::TorqueUnit::NM};
            }
            else ret.torque = {Iqd_measured.q, curr_torque_limit_base_amp, Motion::TorqueUnit::AMP};
            if(const auto& enc = GetPrimaryEncoder())
            {
                ret.speed = {enc.value()->angular_speed_rad_s, curr_speed_limit_base_rad_s, Motion::SpeedUnit::RADS};
                ret.pos = {enc.value()->multi_round_angle_rad, Motion::PosUnit::RAD};
            }
            break;
        }
        case Motion::Ref::OUTPUT:
        {
            if(t == Motion::TorqueUnit::NM && GetConfig().torque_constant_valid())
            {
                const real_t temp = GetConfig().torque_constant() * GetConfig().deduction_ratio();
                ret.torque = {Iqd_measured.q * temp, curr_torque_limit_base_amp * temp, Motion::TorqueUnit::NM};
            }
            else ret.torque = {Iqd_measured.q, curr_torque_limit_base_amp, Motion::TorqueUnit::AMP}; // for Amps torque, we still use current from base.
            if(const auto& enc = GetPrimaryEncoder())
            {
                const real_t temp = 1.0f / GetConfig().deduction_ratio();
                ret.speed = {enc.value()->angular_speed_rad_s * temp, curr_speed_limit_base_rad_s * temp, Motion::SpeedUnit::RADS};
                ret.pos = {enc.value()->multi_round_angle_rad * temp, Motion::PosUnit::RAD};
            }
        }
        default: break;
    }
    ret.ConvertSpeedPosFromDefault(s, p);
}

void FOCMotor::GetTargetMotion(Motion& ret, Motion::Ref r, Motion::TorqueUnit t, Motion::SpeedUnit s, Motion::PosUnit p)
{
    // Motion ret{current_target}; // current_target is stored with Ref == BASE, and other default unit agreements (AMP, RADS, RAD)
    ret = current_target;
    if(ret.ref != Motion::Ref::BASE) // return zero
    {
        ret.Reset();
        return;
    }
    if(r == Motion::Ref::OUTPUT)
    {
        if(t == Motion::TorqueUnit::NM && GetConfig().torque_constant_valid())
        {
            const real_t temp = GetConfig().torque_constant() * GetConfig().deduction_ratio();
            ret.torque = {ret.torque.value * temp, ret.torque.limit * temp, Motion::TorqueUnit::NM}; // BASE -> OUTPUT, Amp -> Nm
        }
        const real_t temp = 1.0f / GetConfig().deduction_ratio();
        ret.speed = {ret.speed.value * temp, ret.speed.limit * temp, Motion::SpeedUnit::RADS}; // BASE -> OUTPUT
        ret.pos = {ret.pos.value * temp, ret.pos.limit * temp, Motion::PosUnit::RAD};
    }
    else // Note that a ref_frame == ELEC will result in the same output as BASE.
    {
        if(t == Motion::TorqueUnit::NM && GetConfig().torque_constant_valid())
        {
            ret.torque = {ret.torque.value * GetConfig().torque_constant(), ret.torque.limit * GetConfig().torque_constant(), Motion::TorqueUnit::NM}; // Amp -> Nm
        }
    }
    ret.ConvertSpeedPosFromDefault(s, p);
}

/// Target motion with ref == ELEC will be IGNORED!
/// \param motion target Motion struct, with reference frame and units stored in std::pair.second
void FOCMotor::SetTargetMotion(Motion& motion)
{
    if(motion.ref == Motion::Ref::ELEC) return;
    motion.ConvertSpeedPosToDefault();
    // now speed and pos are in the default unit agreement, but different reference frame
    if(motion.ref == Motion::Ref::OUTPUT)
    {
        if(GetConfig().deduction_ratio() <= 0.0f) return; // deduction ratio invalid, return
        if(motion.torque.unit == Motion::TorqueUnit::NM)
        {
            real_t temp = 1.0f / GetConfig().deduction_ratio();
            motion.torque.value *= temp; // OUTPUT -> BASE
            motion.torque.limit *= temp;
        }
        motion.speed.value *= GetConfig().deduction_ratio(); // OUTPUT -> BASE
        motion.speed.limit *= GetConfig().deduction_ratio();
        motion.pos.value *= GetConfig().deduction_ratio();
        motion.pos.limit *= GetConfig().deduction_ratio();
    }
    if(motion.torque.unit == Motion::TorqueUnit::NM)
    {
        if(!GetConfig().torque_constant_valid() || GetConfig().torque_constant() <= 0.0f) return; // if received torque target but torque constant is invalid, return
        real_t temp = 1.0f / GetConfig().torque_constant(); // [A/Nm]
        motion.torque = {motion.torque.value * temp, motion.torque.limit * temp, Motion::TorqueUnit::AMP}; // Nm -> A
    }
    motion.ref = Motion::Ref::BASE;
    current_target = motion;
}

void FOCMotor::ResetDefaultConfig()
{
    auto& cfg = GetConfig();
    cfg.clear();
    cfg.set_current_loop_bandwidth(DEFAULT_CURRENT_LOOP_BANDWIDTH);
    cfg.set_calibration_voltage(DEFAULT_CALIBRATION_VOLTAGE);
    cfg.set_calibration_current(DEFAULT_CALIBRATION_CURRENT);
    cfg.set_max_voltage(DEFAULT_PARAM_MOTOR_MAX_VOLTAGE);
    cfg.set_max_current(DEFAULT_PARAM_MOTOR_MAX_CURRENT);
    cfg.set_max_output_speed_rpm(DEFAULT_PARAM_MOTOR_MAX_OUTPUT_SPEED_RPM);
    cfg.set_deduction_ratio(1.0f);
    cfg.set_startup_basic_param_calibration(true);
}

std::optional<FOC::CurrLoopBase*> FOCMotor::GetCurrLoop()
{
    auto currloop = GetTaskByName("CurrLoop");
    if(currloop) curr_loop = std::make_optional(reinterpret_cast<FOC::CurrLoopBase*>(currloop.value()));
    else curr_loop = std::nullopt;
    return curr_loop;
}

std::optional<FOC::SpeedLoopBase*> FOCMotor::GetSpeedLoop()
{
    auto speedloop = GetTaskByName("SpeedLoop");
    if(speedloop) speed_loop = std::make_optional(reinterpret_cast<FOC::SpeedLoopBase*>(speedloop.value()));
    else speed_loop = std::nullopt;
    return speed_loop;
}

}