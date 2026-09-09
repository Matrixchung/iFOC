#include "foc_task_state_machine.hpp"
#include "foc_math.hpp"
#include "../foc_motor.hpp"
#include "../Controller/foc_curr_loop_pi.hpp"
#include "../Controller/foc_speed_loop_pi.hpp"
#include "../Controller/foc_open_loop_controller.hpp"
#include "../Task/foc_task_basic_param_calib.hpp"
#include "../Task/foc_task_extend_param_calib.hpp"
#include "../Task/foc_task_encoder_calib.hpp"
#include "../Task/foc_task_tone_player.hpp"
#include "../Observer/foc_observer_hfi.hpp"
#include "../../../Encoder/encoder_off_axis_base.hpp"
#include "../../../Encoder/encoder_vernier.hpp"

constexpr float OFF_AXIS_ENCODER_VALID_TIMEOUT = 0.5f;

/*
 * TASK LINE: SenseTask -> Encoders... -> EncArbiter -> Park -> ...
 */

#define TRANSITION_OK(new_state) \
do{ last_state = current_state; \
    current_state = new_state;                            \
    EXECUTE(trans_success_cb, new_state, current_state, last_state); \
    return current_state; } while(0)                                 \

#define TRANSITION_FAILED() \
do{ EXECUTE(trans_failure_cb, new_state, current_state, last_state); \
    return current_state; } while(0)                                 \

namespace iFOC::FOC
{
StateMachineTask::StateMachineTask() : Task("StateMachine")
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 2;
    config.stack_depth = 256;
}

void StateMachineTask::InitNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    current_state = MotorState::IDLE;
    last_state = MotorState::IDLE;
    // now initial tasks are appended in INIT()
    // foc->AppendTask(new UpdateSenseTask); // "SenseTask"
    // foc->AppendTask(new EncoderArbiterTask); // "EncArbiter"
    // foc->AppendTask(new ParkTransformTask); // "Park"
    // sleep(50);
    // foc->AppendTask(new WaveGenSVPWM);    // "WaveGen"

    // Off axis encoder
    if(const auto enc = foc->GetPrimaryEncoder())
    {
        if(foc->GetConfig().deduction_ratio() > 1.0f)
        {
            sleep(100); // we will wait for possible auxiliary encoder to boot up correctly.
            if(enc->GetEncoderType() != Encoder::Type::SENSORLESS_ENCODER &&
            foc->GetConfig().pole_pairs_valid() && foc->GetConfig().sensor_zero_offset_valid())
            {
                if(const auto off_axis = foc->GetEncoderByName("EncOffAxis"))
                {
                    float off_axis_encoder_valid_timer = 0.0f;
                    while(!off_axis->IsResultValid() || !enc->IsResultValid())
                    {
                        off_axis_encoder_valid_timer += 0.01f;
                        if(off_axis_encoder_valid_timer >= OFF_AXIS_ENCODER_VALID_TIMEOUT) break;
                        sleep(10);
                    }
                    if(off_axis->IsResultValid() && enc->IsResultValid())
                    {
                        float min_error = std::numeric_limits<float>::infinity();
                        int target_full_rotations = 0;
                        const int k_max = (int)ceilf(foc->GetConfig().deduction_ratio()) - 1;
                        for(int k = 0; k <= k_max; ++k)
                        {
                            // get candidate output_single_round_rad
                            const float candidate = normalize_rad(((float)k * PI2 + enc->compensated_single_round_angle_rad) / foc->GetConfig().deduction_ratio());
                            const float abs_error = ABS(normalize_rad_pm_pi(candidate - off_axis->compensated_single_round_angle_rad));
                            if(abs_error < min_error)
                            {
                                min_error = abs_error;
                                target_full_rotations = k;
                            }
                        }
                        enc->full_rotations = target_full_rotations;
                    }
                }
                else if(const auto vernier = foc->GetEncoderByName("EncVernier"))
                {
                    float off_axis_encoder_valid_timer = 0.0f;
                    while(!vernier->IsResultValid() || !enc->IsResultValid())
                    {
                        off_axis_encoder_valid_timer += 0.01f;
                        if(off_axis_encoder_valid_timer >= OFF_AXIS_ENCODER_VALID_TIMEOUT) break;
                        sleep(10);
                    }
                    const float gear_ratio = ((Encoder::EncoderVernier*)vernier)->GetGearRatio();
                    if(vernier->IsResultValid() && enc->IsResultValid())
                    {
                        float min_error = PI2;
                        int target_full_rotations = 0;
                        const int k_max = (int)ceilf(foc->GetConfig().deduction_ratio()) - 1;
                        for(int k = 0; k <= k_max; ++k)
                        {
                            const float theta = enc->compensated_single_round_angle_rad + PI2 * (float)k;
                            const float beta_pred = normalize_rad((-gear_ratio) * theta);
                            const float abs_error = ABS(normalize_rad_pm_pi(beta_pred - vernier->compensated_single_round_angle_rad));
                            if(abs_error < min_error)
                            {
                                min_error = abs_error;
                                target_full_rotations = k;
                            }
                        }
                        enc->full_rotations = target_full_rotations;
                    }
                }
            }
        }
        else
        {
            foc->RemoveEncoderByName("EncVernier");
            foc->RemoveEncoderByName("EncOffAxis");
        }
    }

    foc->SetControlMode(MotorControlMode::CTRL_MODE_CURRENT); // current mode init

    // Here we are in IDLE.
    // Play the beep first, but with a proper basic parameter set to avoid electrical misconfiguration
    if(!CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
        BoardConfig().GetConfig().play_startup_tone())
    {
        auto* tone_player = new TonePlayerTask;
        if(foc->InsertTaskBeforeName("WaveGen", tone_player) == FuncRetCode::OK)
        {
            tone_player->PlaySound({1200.0f, 1650.0f, 2200.0f} ,0.25f, true);
            while(!tone_player->IsCompleted()) sleep(100);
            foc->RemoveTaskByName("TonePlayer");
            foc->Disarm();
        }
        else delete tone_player;
        // while(foc->GetTaskByName("TonePlayer")) sleep(100);
    }
    if(foc->GetConfig().startup_sequence_enabled()) RequestState(MotorState::STARTUP_SEQUENCE);
}

void StateMachineTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    if(isr_request_state_called)
    {
        RequestState(isr_requested_state);
        isr_requested_state = MotorState::IDLE;
        isr_request_state_called = false;
    }
    switch(current_state)
    {
        case MotorState::STARTUP_SEQUENCE:
        {
            if(foc->GetConfig().startup_basic_param_calibration())
            {
                if(CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION))
                {
                    RequestState(MotorState::BASIC_PARAM_CALIBRATION);
                    break;
                }
            }
            if(foc->GetConfig().startup_encoder_index_search())
            {
                if(CheckStateRequirement(MotorState::ENCODER_INDEX_SEARCH))
                {
                    RequestState(MotorState::ENCODER_INDEX_SEARCH);
                    break;
                }
            }
            if(foc->GetConfig().startup_encoder_calibration())
            {
                if(CheckStateRequirement(MotorState::ENCODER_CALIBRATION))
                {
                    RequestState(MotorState::ENCODER_CALIBRATION);
                    break;
                }
            }
            if(foc->GetConfig().startup_extend_param_calibration())
            {
                if(CheckStateRequirement(MotorState::EXTEND_PARAM_CALIBRATION))
                {
                    RequestState(MotorState::EXTEND_PARAM_CALIBRATION);
                    break;
                }
            }
            if(foc->GetConfig().startup_sensored_closed_loop())
            {
                if(CheckStateRequirement(MotorState::SENSORED_CLOSED_LOOP_CONTROL))
                {
                    RequestState(MotorState::SENSORED_CLOSED_LOOP_CONTROL);
                    break;
                }
                foc->DisarmWithError(MotorError::STARTUP_SEQ_REQUIREMENTS_UNMET);
                break;
            }
            if(foc->GetConfig().startup_sensorless_closed_loop())
            {
                if(CheckStateRequirement(MotorState::SENSORLESS_CLOSED_LOOP_CONTROL))
                {
                    RequestState(MotorState::SENSORLESS_CLOSED_LOOP_CONTROL);
                    break;
                }
                foc->DisarmWithError(MotorError::STARTUP_SEQ_REQUIREMENTS_UNMET);
                break;
            }
            RequestState(MotorState::IDLE);
            break;
        }
        case MotorState::BASIC_PARAM_CALIBRATION:
        {
            if(!foc->GetTaskByName("BasicParam") &&
                CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION))
                    foc->InsertTaskBeforeName("WaveGen", new BasicParamCalibTask);
            sleep(100);
            break;
        }
        case MotorState::ENCODER_INDEX_SEARCH:
        {
            // sleep(100);
            RequestState(MotorState::IDLE); // TODO
            break;
        }
        case MotorState::ENCODER_CALIBRATION:
        {
            if(!foc->GetTaskByName("EncCalib") &&
               CheckStateRequirement(MotorState::ENCODER_CALIBRATION))
                foc->InsertTaskBeforeName("WaveGen", new EncoderCalibTask);
            sleep(100);
            break;
        }
        case MotorState::EXTEND_PARAM_CALIBRATION:
        {
            if(!foc->GetTaskByName("ExtCalib") &&
               CheckStateRequirement(MotorState::EXTEND_PARAM_CALIBRATION))
                foc->InsertTaskBeforeName("WaveGen", new ExtendParamCalibTask);
            sleep(100);
            break;
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
        case MotorState::OPEN_LOOP_VELOCITY_CONTROL:
        {
            sleep(10);
            break;
        }
        case MotorState::IDLE: sleep(10); break;
        default: RequestState(MotorState::IDLE); break; // For unimplemented states, go back to IDLE.
    }
}

bool StateMachineTask::CheckStateRequirement(const MotorState new_state)
{
    const auto foc = GetMotor<FOCMotor>();
    switch(new_state)
    {
        case MotorState::BASIC_PARAM_CALIBRATION:
        {
            // prevent re-entering
            // if(foc->GetTaskByName("BasicParam")) return false;
            while(foc->GetTaskByName("BasicParam")) sleep(10);
            return !foc->GetConfig().phase_resistance_valid() ||
                    !foc->GetConfig().phase_inductance_valid() ||
                    !foc->GetConfig().flux_linkage_valid();
        }
        case MotorState::ENCODER_INDEX_SEARCH:
        {
            while(foc->GetTaskByName("IndexSearch")) sleep(10);
            if(const auto enc = foc->GetPrimaryEncoder())
            {
                return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
                        enc->GetEncoderType() == Encoder::Type::INCREMENTAL_ENCODER &&
                        !enc->IsResultValid();
            }
            return false;
        }
        case MotorState::ENCODER_CALIBRATION:
        {
            while(foc->GetTaskByName("EncCalib")) sleep(10);
            if(const auto enc = foc->GetPrimaryEncoder())
            {
                return !foc->GetConfig().sensor_direction_valid() ||
                        !foc->GetConfig().pole_pairs_valid() ||
                        !foc->GetConfig().sensor_zero_offset_valid();
            }
            return false;
        }
        case MotorState::EXTEND_PARAM_CALIBRATION:
        {
            while(foc->GetTaskByName("ExtCalib")) sleep(10);
            const auto enc = foc->GetEncoderByName("EncOffAxis");
            return CheckStateRequirement(MotorState::SENSORED_CLOSED_LOOP_CONTROL) &&
                    ((enc &&
                        ((Encoder::EncoderOffAxisBase*)enc)->IsConnected() &&
                        !((Encoder::EncoderOffAxisBase*)enc)->IsCalibrated() &&
                        foc->GetPrimaryEncoder() &&
                        foc->GetConfig().deduction_ratio() > 1.0f)
#if FOC_ANTICOGGING_AVAILABLE
                        || (foc->GetConfig().anticogging_base_pos_err_deg() > 0.0f &&
                            foc->GetConfig().anticogging_base_vel_err_rpm() > 0.0f &&
                            foc->GetConfig().vel_kp() > 0.0f &&
                            foc->GetConfig().vel_ki() > 0.0f &&
                            foc->GetConfig().pos_kp() > 0.0f &&
                            foc->anticogging_lut.getTableSize() != ANTICOGGING_LUT_POINTS)
#endif
                     );
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        {
            return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
                   !CheckStateRequirement(MotorState::ENCODER_INDEX_SEARCH) &&
                   !CheckStateRequirement(MotorState::ENCODER_CALIBRATION);
        }
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
        {
            return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION);
        }
        case MotorState::OPEN_LOOP_VELOCITY_CONTROL:
        {
            // while(foc->GetTaskByName("IndexSearch")) sleep(10);
            return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) && foc->GetConfig().pole_pairs_valid();
        }
        default: return false;
    }
}

MotorState StateMachineTask::BackToLastState()
{
    return RequestState(last_state);
}

MotorState StateMachineTask::RequestState(const MotorState new_state)
{
    if(xPortIsInsideInterrupt())
    {
        isr_request_state_called = true;
        isr_requested_state = new_state;
        return new_state; // latch the request to RTOS task
    }
    const auto foc = GetMotor<FOCMotor>();
    if(current_state == new_state) TRANSITION_FAILED();
    switch(new_state)
    {
        case MotorState::IDLE:
        {
            // FIX: when debugging gate drivers with 'GetDriver()->SetOutput3CHPu(x, y, z)' in the main code,
            //      foc->Disarm() will continuously disarm the driver.
            if(foc->IsArmed()) foc->Disarm();
            // FIX: when switching state from closed_loop_control modes back to IDLE, the speed_loop & curr_loop are not handled correctly.
            foc->RemoveTaskByName("CurrLoop");
            foc->RemoveTaskByName("SpeedLoop");
            foc->RemoveTaskByName("OpenLoop");
            foc->RemoveTaskByName("HFIMain");
            foc->RemoveTaskByName("TonePlayer");
            auto current_target = foc->GetTargetMotionStruct(Motion::Ref::BASE,
                                                             Motion::TorqueUnit::AMP,
                                                             Motion::SpeedUnit::RPM,
                                                             Motion::PosUnit::DEG);
            // reset current torque & speed target, but keeping pos target
            current_target.torque.value = 0.0f;
            current_target.speed.value = 0.0f;
            foc->SetTargetMotion(current_target);
            foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION); // reset to default mode
            TRANSITION_OK(new_state);
        }
        case MotorState::STARTUP_SEQUENCE:
        {
            if(foc->GetError() != to_underlying(MotorError::NONE)) TRANSITION_FAILED();
            if((current_state == MotorState::IDLE) || // Situation #1: Initial, from IDLE state
               (last_state == MotorState::STARTUP_SEQUENCE && ( // Situation #2: From Startup Sequences' call to main sequence
                       to_underlying(current_state) >= to_underlying(MotorState::BASIC_PARAM_CALIBRATION) &&
                       to_underlying(current_state) <= to_underlying(MotorState::EXTEND_PARAM_CALIBRATION)))
               )
            {
                TRANSITION_OK(new_state);
            }
            TRANSITION_FAILED();
        }
        case MotorState::BASIC_PARAM_CALIBRATION:
        case MotorState::ENCODER_INDEX_SEARCH:
        case MotorState::ENCODER_CALIBRATION:
        case MotorState::EXTEND_PARAM_CALIBRATION:
        {
            if(foc->GetError() != to_underlying(MotorError::NONE)) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE || current_state == MotorState::IDLE)
            {
                if(CheckStateRequirement(new_state))
                {
                    foc->RemoveTaskByName("TonePlayer");
                    TRANSITION_OK(new_state);
                }
            }
            TRANSITION_FAILED();
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        {
            if(foc->GetError() != to_underlying(MotorError::NONE)) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE ||
                current_state == MotorState::IDLE ||
                current_state == MotorState::SENSORLESS_CLOSED_LOOP_CONTROL)
            {
                if(CheckStateRequirement(new_state))
                {
                    foc->RemoveTaskByName("TonePlayer");
                    foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
                    foc->InsertTaskBeforeName("CurrLoop", new SpeedLoopPI);
                    auto current_target = foc->GetTargetMotionStruct(Motion::Ref::BASE,
                                                             Motion::TorqueUnit::AMP,
                                                             Motion::SpeedUnit::RADS,
                                                             Motion::PosUnit::RAD);
                    const auto current_motion = foc->GetCurrentMotionStruct(current_target);
                    // reset current torque & speed target, and sync pos target with current state.
                    // (to avoid unexpected movement during IDLE -> CLOSED_LOOP)
                    current_target.torque.value = 0.0f;
                    current_target.speed.value = 0.0f;
                    current_target.pos.value = current_motion.pos.value;
                    foc->SetTargetMotion(current_target);
                    foc->SetControlMode(MotorControlMode::CTRL_MODE_CURRENT); // current mode init
                    foc->Arm();
                    TRANSITION_OK(new_state);
                }
            }
            TRANSITION_FAILED();
        }
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
        {
            if(foc->GetError() != to_underlying(MotorError::NONE)) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE ||
                current_state == MotorState::IDLE ||
                current_state == MotorState::SENSORED_CLOSED_LOOP_CONTROL)
            {
                if(CheckStateRequirement(new_state))
                {
                    foc->RemoveTaskByName("TonePlayer");
                    foc->InsertTaskAfterName("SenseTask", new ObserverHFI); // HFI injector acts as a modifier to Ialphabeta_measured, before park
                    // foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
                    foc->Arm();
                    TRANSITION_OK(new_state);
                }
            }
            TRANSITION_FAILED();
        }
        case MotorState::OPEN_LOOP_VELOCITY_CONTROL:
        {
            if(foc->GetError() != to_underlying(MotorError::NONE)) TRANSITION_FAILED();
            if(current_state == MotorState::IDLE)
            {
                if(CheckStateRequirement(new_state))
                {
                    foc->RemoveTaskByName("TonePlayer");
                    foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
                    foc->InsertTaskBeforeName("CurrLoop", new OpenLoopController);
                    foc->Arm();
                    TRANSITION_OK(new_state);
                }
            }
            TRANSITION_FAILED();
        }
        default: TRANSITION_FAILED();
    }
}

}