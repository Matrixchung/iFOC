#include "foc_task_state_machine.hpp"
#include "foc_math.hpp"
#include "../foc_motor.hpp"
#include "./Controller/foc_curr_loop_pi.hpp"
#include "./Controller/foc_speed_loop_pi.hpp"
#include "./Task/foc_task_update_sense.hpp"
#include "./Task/foc_task_encoder_arbiter.hpp"
#include "./Task/foc_task_basic_param_calib.hpp"
#include "./Task/foc_task_encoder_calib.hpp"
#include "./Task/foc_task_tone_player.hpp"
#include "./WaveGenerator/foc_wave_gen_svpwm.hpp"

#define foc GetMotor<FOCMotor>()

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
    current_state = MotorState::IDLE;
    last_state = MotorState::IDLE;
    foc->AppendTask(new EncoderArbiterTask); // "EncArbiter"
    foc->AppendTask(new UpdateSenseTask); // "SenseTask"
    sleep(50);
    foc->AppendTask(new WaveGenSVPWM);    // "WaveGen"
    // Here we are in IDLE.
    // Play the beep first, but with a proper basic parameter set to avoid electrical misconfiguration
    if(!CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
        BoardConfig.GetConfig().play_startup_tone())
    {
        auto tone_player = new TonePlayerTask;
        if(foc->InsertTaskBeforeName("WaveGen", tone_player) == FuncRetCode::OK)
            tone_player->PlaySound({1200.0f, 1650.0f, 2200.0f} ,0.25f, true);
    }
    while(foc->GetTaskByName("TonePlayer")) sleep(100);
    foc->Disarm();
    if(foc->GetConfig().startup_sequence_enabled()) RequestState(MotorState::STARTUP_SEQUENCE);
}

void StateMachineTask::UpdateNormal()
{
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
                foc->DisarmWithError(MotorError::STARTUP_SENSORED_CLOSE_LOOP_REQ_NOT_MET);
                break;
            }
            if(foc->GetConfig().startup_sensorless_closed_loop())
            {
                if(CheckStateRequirement(MotorState::SENSORLESS_CLOSED_LOOP_CONTROL))
                {
                    RequestState(MotorState::SENSORLESS_CLOSED_LOOP_CONTROL);
                    break;
                }
                foc->DisarmWithError(MotorError::STARTUP_SENSORLESS_CLOSE_LOOP_REQ_NOT_MET);
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
            RequestState(MotorState::IDLE); // TODO
            break;
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        {
            sleep(10);
            break;
        }
        case MotorState::IDLE: sleep(10); break;
        default: RequestState(MotorState::IDLE); break; // For unimplemented states, go back to IDLE.
    }
}

bool StateMachineTask::CheckStateRequirement(MotorState new_state)
{
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
            if(auto enc = foc->GetPrimaryEncoder())
            {
                return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
                        enc.value()->GetEncoderType() == Encoder::Type::INCREMENTAL_ENCODER &&
                        !enc.value()->IsResultValid();
            }
            return false;
        }
        case MotorState::ENCODER_CALIBRATION:
        {
            while(foc->GetTaskByName("EncCalib")) sleep(10);
            if(auto enc = foc->GetPrimaryEncoder())
            {
                return !foc->GetConfig().sensor_direction_valid() ||
                        !foc->GetConfig().pole_pairs_valid() ||
                        !foc->GetConfig().sensor_zero_offset_valid();
            }
            return false;
        }
        case MotorState::EXTEND_PARAM_CALIBRATION:
        {
            while(foc->GetTaskByName("ExtParamCalib")) sleep(10);
            // return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
            //         !CheckStateRequirement(MotorState::ENCODER_INDEX_SEARCH) &&
            //         !CheckStateRequirement(MotorState::ENCODER_CALIBRATION) &&
            //         (!foc->GetConfig().flux_linkage_valid());
            return false;
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        {
            return !CheckStateRequirement(MotorState::BASIC_PARAM_CALIBRATION) &&
                   !CheckStateRequirement(MotorState::ENCODER_INDEX_SEARCH) &&
                   !CheckStateRequirement(MotorState::ENCODER_CALIBRATION);
        }
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
        {
            return false; // TODO
        }
        default: return false;
    }
}

MotorState StateMachineTask::BackToLastState()
{
    return RequestState(last_state);
}

MotorState StateMachineTask::RequestState(MotorState new_state)
{
    if(current_state == new_state) TRANSITION_FAILED();
    switch(new_state)
    {
        case MotorState::IDLE:
        {
            foc->Disarm();
            TRANSITION_OK(new_state);
        }
        case MotorState::STARTUP_SEQUENCE:
        {
            if(foc->GetError() != MotorError::NONE) TRANSITION_FAILED();
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
            if(foc->GetError() != MotorError::NONE) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE || current_state == MotorState::IDLE)
            {
                if(CheckStateRequirement(new_state)) TRANSITION_OK(new_state);
            }
            TRANSITION_FAILED();
        }
        case MotorState::SENSORED_CLOSED_LOOP_CONTROL:
        {
            if(foc->GetError() != MotorError::NONE) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE ||
                current_state == MotorState::IDLE ||
                current_state == MotorState::SENSORLESS_CLOSED_LOOP_CONTROL)
            {
                if(CheckStateRequirement(new_state))
                {
                    foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
                    foc->InsertTaskBeforeName("CurrLoop", new SpeedLoopPI);
                    foc->Arm();
                    TRANSITION_OK(new_state);
                }
            }
            TRANSITION_FAILED();
        }
        case MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
        {
            if(foc->GetError() != MotorError::NONE) TRANSITION_FAILED();
            if(current_state == MotorState::STARTUP_SEQUENCE ||
                current_state == MotorState::IDLE ||
                current_state == MotorState::SENSORED_CLOSED_LOOP_CONTROL)
            {
                if(CheckStateRequirement(new_state)) TRANSITION_OK(new_state);
            }
            TRANSITION_FAILED();
        }
        default: TRANSITION_FAILED();
    }
}

}