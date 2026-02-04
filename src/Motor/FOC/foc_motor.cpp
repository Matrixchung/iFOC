#include "foc_motor.hpp"
#include "./Controller/foc_curr_loop_base.hpp"
#include "./Controller/foc_speed_loop_base.hpp"
#include "./Task/foc_task_update_sense.hpp"
#include "./Task/foc_task_park_transform.hpp"
#include "./Task/foc_task_encoder_arbiter.hpp"
#include "./WaveGenerator/foc_wave_gen_svpwm.hpp"

#define DEFAULT_NODE_ID (255UL)
#define DEFAULT_CAN_HEARTBEAT_INTERVAL_MS (1000)
#define DEFAULT_CAN_FEEDBACK_INTERVAL_MS  (2)
#define DEFAULT_CURRENT_LOOP_BANDWIDTH (1000.0f)
#define DEFAULT_CALIBRATION_VOLTAGE (1.0f)
#define DEFAULT_CALIBRATION_CURRENT (1.0f)
#define DEFAULT_PARAM_MOTOR_MAX_VOLTAGE (12.0f)
#define DEFAULT_PARAM_MOTOR_MAX_CURRENT (10.0f)
#define DEFAULT_PARAM_MOTOR_MAX_OUTPUT_SPEED_RPM (1000.0f)
#define DEFAULT_PARAM_MOTOR_SENSOR_SPEED_F_LP (200.0f)

using namespace iFOC::FOC;

namespace iFOC
{
FuncRetCode FOCMotor::Init(const bool initTIM)
{
    FuncRetCode result = FuncRetCode::HARDWARE_ERROR;
    /// WARN Static global variable must be externed in header, instead of directly defined in header
    const auto& board = BoardConfig().GetConfig();
    if(!driver)
    {
        ThrowError(MotorError::DRIVER_COMPONENT_MISSING);
        result = FuncRetCode::INVALID_INPUT;
        goto error;
    }
    if(!curr_sense)
    {
        ThrowError(MotorError::CURR_SENSE_COMPONENT_MISSING);
        result = FuncRetCode::INVALID_INPUT;
        goto error;
    }
    if(!bus_sense)
    {
        ThrowError(MotorError::BUS_SENSE_COMPONENT_MISSING);
        result = FuncRetCode::INVALID_INPUT;
        goto error;
    }
    // Config safety check, board config
    if (board.get_bus_overvoltage_limit() <= 1.0f ||
        board.get_bus_undervoltage_limit() >= board.get_bus_overvoltage_limit() ||
        board.get_bus_max_positive_current() <= 0.01f ||
        board.get_bus_max_negative_current() >= -0.01f)
    {
        ThrowError(MotorError::CONFIG_BOARD_CONFIG_INVALID);
        result = FuncRetCode::PARAM_OUT_BOUND;
        goto error;
    }
    // Config safety check, motor config
    if(config.ReadNVMConfig() != FuncRetCode::OK)
    {
        ThrowError(MotorError::SYSTEM_MOTOR_CONFIG_READ_ERROR);
    }
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
    result = bus_sense->Init();
    if(result != FuncRetCode::OK)
    {
        if(result == FuncRetCode::PARAM_OUT_BOUND) ThrowError(MotorError::CONFIG_BUS_SENSE_CONFIG_INVALID);
        else if(result == FuncRetCode::CRC_MISMATCH) ThrowError(MotorError::BUS_SENSE_DEV_ID_MISMATCH);
        else if(result == FuncRetCode::HARDWARE_ERROR) ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
        ThrowError(MotorError::BUS_SENSE_INIT_FAILED);
        goto error;
    }
    result = bus_sense->Update();
    if(result != FuncRetCode::OK)
    {
        ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
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
        if(result == FuncRetCode::HARDWARE_ERROR) ThrowError(MotorError::DRIVER_COMMUNICATION_ERROR);
        else if(result == FuncRetCode::CRC_MISMATCH) ThrowError(MotorError::DRIVER_DEV_ID_MISMATCH);
        else if(result == FuncRetCode::PARAM_OUT_BOUND) ThrowError(MotorError::CONFIG_CURR_SENSE_CONFIG_INVALID);
        driver->DisableAllOutputs();
        ThrowError(MotorError::DRIVER_INIT_FAILED);
        goto error;
    }
    if(const auto ind = GetIndicator()) ind->Init();
    AppendTask(&this->state_machine);
    AppendTask(new UpdateSenseTask); // "SenseTask"
    AppendTask(new EncoderArbiterTask); // "EncArbiter"
    AppendTask(new ParkTransformTask); // "Park"
    HAL::DelayMs(50);
    AppendTask(new WaveGenSVPWM);    // "WaveGen"
    return FuncRetCode::OK;
error:
    if(const auto ind = GetIndicator())
    {
        ind->Init();
        ind->SetRGB(255, 0, 0);
        // HAL::DelayMs(50);
        // ind->SetRGB(255, 0, 0);
        // HAL::DelayMs(50);
        // ind->SetRGB(255, 0, 0);
    }
    return result;
}

bool FOCMotor::Arm()
{
    if(const auto& curr = GetCurrLoop()) curr->ResetCurrLoop();
    if(const auto& speed = GetSpeedLoop()) speed->ResetSpeedLoop();
    if(error == to_underlying(MotorError::NONE))
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
    return error == to_underlying(MotorError::NONE);
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
    _is_ramping_motion = false; // reset ramping
    _is_trajectory_motion = false; // reset trajectory
    current_target.Reset();
    Iqd_target = {0.0f, 0.0f};
    Uqd_target = {0.0f, 0.0f};
    traj_controller.Reset();
    _traj_final_target_motion.Reset();
    _ramped_diff_motion.Reset();
    _ramped_start_target_motion.Reset();
}

void FOCMotor::DisarmWithError(MotorError e)
{
    Disarm();
    state_machine.RequestState(MotorState::IDLE);
    ThrowError(e);
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
                float spd = enc->angular_speed_rad_s, pos = enc->multi_round_angle_rad;
                if(enc->GetEncoderType() == Encoder::Type::SENSORLESS_ENCODER)
                {
                    if(GetConfig().pole_pairs_valid() && GetConfig().pole_pairs() > 0)
                    {
                        spd /= GetConfig().pole_pairs();
                        pos /= GetConfig().pole_pairs();
                    }
                }
                ret.speed = {spd, curr_speed_limit_base_rad_s, Motion::SpeedUnit::RADS};
                ret.pos = {pos, Motion::PosUnit::RAD};
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
                float spd = enc->angular_speed_rad_s, pos = enc->multi_round_angle_rad;
                if(enc->GetEncoderType() == Encoder::Type::SENSORLESS_ENCODER)
                {
                    if(GetConfig().pole_pairs_valid() && GetConfig().pole_pairs() > 0)
                    {
                        spd /= GetConfig().pole_pairs();
                        pos /= GetConfig().pole_pairs();
                    }
                }
                const real_t temp = 1.0f / GetConfig().deduction_ratio();
                ret.speed = {spd * temp, curr_speed_limit_base_rad_s * temp, Motion::SpeedUnit::RADS};
                ret.pos = {pos * temp, Motion::PosUnit::RAD};
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
    // constrain speed & current
    motion.speed.value = _constrain(motion.speed.value, -config_max_base_speed_rad_s, config_max_base_speed_rad_s);
    motion.speed.limit = _constrain(motion.speed.limit, 0.0f, config_max_base_speed_rad_s);
    motion.torque.value = _constrain(motion.torque.value, -config_max_current, config_max_current);
    motion.torque.limit = _constrain(motion.torque.limit, 0.0f, config_max_current);
    motion.ref = Motion::Ref::BASE;
    current_target = motion;
    _is_ramping_motion = false; // If ramping, will be overrided by SetRampedMotion() later.
    _is_trajectory_motion = false;
}

void FOCMotor::SetTrajectoryTargetMotion(Motion& motion, bool is_s_curve)
{
    if(_is_ramping_motion || !IsArmed()) return;
    // Given that trajectory parameters: traj_output_speed/accel/decel_limit_rpm
    // are set under OUTPUT reference with RPM/RPM^2 unit,
    // we should first transform the parameters to BASE reference, with RADS unit.
    if(motion.ref == Motion::Ref::ELEC) return; // ignore ELEC reference.
    if(GetConfig().deduction_ratio() <= 0.0f) return; // deduction ratio invalid, return

    real_t traj_base_speed_lim_rads = RPM2RAD(GetConfig().traj_output_speed_limit_rpm() * GetConfig().deduction_ratio(), 1);
    real_t traj_base_accel_lim_rads2 = RPM2RAD(GetConfig().traj_output_accel_limit_rpm() * GetConfig().deduction_ratio(), 1);
    real_t traj_base_decel_lim_rads2 = RPM2RAD(GetConfig().traj_output_decel_limit_rpm() * GetConfig().deduction_ratio(), 1);
    if(traj_base_speed_lim_rads <= 0.0f || traj_base_accel_lim_rads2 <= 0.0f || traj_base_decel_lim_rads2 <= 0.0f) return; // settings invalid, return

    motion.ConvertSpeedPosToDefault();
    if(motion.ref == Motion::Ref::OUTPUT)
    {
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
    motion.ref = Motion::Ref::BASE;
    // now we have BASE ref, with RADS speed & RAD pos.
    const auto current_motion = GetCurrentMotionStruct(motion);
    traj_controller.PlanTrajectory(motion.pos.value,
                                   current_motion.pos.value,
                                   current_motion.speed.value,
                                   traj_base_speed_lim_rads,
                                   traj_base_accel_lim_rads2,
                                   traj_base_decel_lim_rads2, is_s_curve);
    _traj_final_target_motion = motion;
    _is_trajectory_motion = true;
}

FuncRetCode FOCMotor::AppendEncoder(Encoder::EncoderBase* encoder)
{
    const auto ret = MotorBase::AppendEncoder(encoder);
    if(ret == FuncRetCode::OK)
    {
        if(GetConfig().sensor_direction_valid())
        {
            if(GetConfig().sensor_direction_clockwise() == false) // we need clockwise, so reverse primary encoder sign.
            {
                if(auto enc = GetPrimaryEncoder(); enc && enc == encoder) enc->SetSign(-1 * enc->GetSign());
            }
        }
    }
    return ret;
}

void FOCMotor::ResetDefaultConfig()
{
    auto& cfg = GetConfig();
    cfg.clear();
    cfg.set_node_id(DEFAULT_NODE_ID);
    cfg.set_can_heartbeat_interval_ms(DEFAULT_CAN_HEARTBEAT_INTERVAL_MS);
    cfg.set_can_feedback_interval_ms(DEFAULT_CAN_FEEDBACK_INTERVAL_MS);
    cfg.set_current_loop_bandwidth(DEFAULT_CURRENT_LOOP_BANDWIDTH);
    cfg.set_calibration_voltage(DEFAULT_CALIBRATION_VOLTAGE);
    cfg.set_calibration_current(DEFAULT_CALIBRATION_CURRENT);
    cfg.set_max_voltage(DEFAULT_PARAM_MOTOR_MAX_VOLTAGE);
    cfg.set_max_current(DEFAULT_PARAM_MOTOR_MAX_CURRENT);
    cfg.set_max_output_speed_rpm(DEFAULT_PARAM_MOTOR_MAX_OUTPUT_SPEED_RPM);
    cfg.set_sensor_speed_f_lp(DEFAULT_PARAM_MOTOR_SENSOR_SPEED_F_LP);
    cfg.set_enable_harmonic_suppression(false);
    cfg.set_deduction_ratio(1.0f);
    cfg.set_startup_basic_param_calibration(true);
}

CurrLoopBase* FOCMotor::GetCurrLoop()
{
    auto currloop = GetTaskByName("CurrLoop");
    // if(currloop) curr_loop = std::make_optional(reinterpret_cast<FOC::CurrLoopBase*>(currloop));
    // else curr_loop = std::nullopt;
    // return curr_loop;
    if(currloop) return reinterpret_cast<CurrLoopBase*>(currloop);
    return nullptr;
}

SpeedLoopBase* FOCMotor::GetSpeedLoop()
{
    auto speedloop = GetTaskByName("SpeedLoop");
    // if(speedloop) speed_loop = std::make_optional(reinterpret_cast<FOC::SpeedLoopBase*>(speedloop));
    // else speed_loop = std::nullopt;
    // return speed_loop;
    if(speedloop) return reinterpret_cast<FOC::SpeedLoopBase*>(speedloop);
    return nullptr;
}

}