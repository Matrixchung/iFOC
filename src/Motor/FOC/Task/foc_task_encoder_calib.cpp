#include "foc_task_encoder_calib.hpp"
#include "foc_curr_loop_pi.hpp"

// Encoder-related calibration
#include "encoder_mt6835.hpp"
#include "trajectory_controller.hpp"

#define foc GetMotor<FOCMotor>()

namespace iFOC::FOC
{
EncoderCalibTask::EncoderCalibTask() : Task("EncCalib")
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 5;
    config.stack_depth = 256;
}

static constexpr float MIN_ANGLE_DETECT_MOVEMENT = PI2 / 101.0f;

void EncoderCalibTask::InitNormal()
{
    const auto& enc = foc->GetPrimaryEncoder();
    if(!enc.has_value())
    {
        foc->DisarmWithError(MotorError::PRIMARY_SENSOR_NOT_FOUND);
        foc->RemoveTaskByName(GetName());
        return;
    }
    if(!enc.value()->IsResultValid())
    {
        foc->DisarmWithError(MotorError::PRIMARY_SENSOR_RESULT_INVALID);
        foc->RemoveTaskByName(GetName());
        return;
    }
    // Bypass Encoder Arbiter
    foc->BypassTaskByName("EncArbiter", "SpeedLoop");
    foc->elec_angle_rad = 0.0f;
    foc->elec_omega_rad_s = 0.0f;
    foc->Uqd_target = {0.0f, 0.0f};
    foc->Iqd_target = {0.0f, 0.0f};
    foc->Arm();
    sleep(100);
    // Because we've finished basic parameter calibration,
    // we are going to add current loop here
    foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
    // Pre-locating, Uq = u, theta = 270 ~ Ud = u, theta = 0
    foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
    sleep(500);
    foc->Iqd_target = {0.0f, 0.0f};
    sleep(200);
    stage = EstStage::NONE;
}

void EncoderCalibTask::UpdateNormal()
{
    switch(stage)
    {
        case EstStage::NONE:
        {
            foc->Iqd_target = {0.0f, 0.0f};
            if(!foc->GetConfig().sensor_direction_valid())
            {
                foc->Arm();
                stage = EstStage::SENSOR_DIRECTION_TESTING;
                break;
            }
            if(!foc->GetConfig().pole_pairs_valid())
            {
                foc->Arm();
                stage = EstStage::POLE_PAIRS_TESTING;
                break;
            }
            if(!foc->GetConfig().sensor_zero_offset_valid())
            {
                foc->Arm();
                stage = EstStage::SENSOR_ZERO_OFFSET_TESTING;
                break;
            }
            if(!is_sensor_custom_calibrated && stage_passed > 0)
            {
                foc->Arm();
                stage = EstStage::SENSOR_CUSTOM_CALIBRATING;
                break;
            }
            foc->Disarm();
            foc->RemoveTaskByName("CurrLoop");
            foc->UnbypassTaskByName("EncArbiter");
            foc->state_machine.BackToLastState();
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::SENSOR_DIRECTION_TESTING:
        {
            const auto encoder = foc->GetPrimaryEncoder().value();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_NOT_FOUND);
                foc->RemoveTaskByName(GetName());
                break;
            }
            for(int i = 0; i < 500; i++)
            {
                float angle_rad = PI2 * (float)i / 500.0f;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                sleep(2);
            }
            float mid_angle = 0.0f;
            mid_angle = encoder->single_round_angle_rad;
            for(int i = 499; i >= 0; i--)
            {
                float angle_rad = PI2 * (float)i / 500.0f;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                sleep(2);
            }
            foc->Iqd_target = {0.0f, 0.0f};
            sleep(200);
            float end_angle = 0.0f;
            end_angle = encoder->single_round_angle_rad;
            float moved = std::fabsf(mid_angle - end_angle);
            if(moved < MIN_ANGLE_DETECT_MOVEMENT)
            {
                foc->DisarmWithError(MotorError::MOTOR_FAILED_TO_ROTATE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            if(mid_angle < end_angle)
            {
                foc->GetConfig().set_sensor_direction_clockwise(false);
                // we need clockwise, so reverse primary encoder sign.
                encoder->SetSign(-1 * encoder->GetSign());
            }
            else
            {
                foc->GetConfig().set_sensor_direction_clockwise(true);
            }
            foc->GetConfig().set_sensor_direction_valid(true);
            stage = EstStage::NONE;
            stage_passed++;
            break;
        }
        case EstStage::POLE_PAIRS_TESTING:
        {
            const auto encoder = foc->GetPrimaryEncoder().value();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_NOT_FOUND);
                foc->RemoveTaskByName(GetName());
                break;
            }
            sleep(100);
            foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
            foc->elec_angle_rad = 0.0f;
            sleep(1000); // move motor to elec angle 0
            float angle_begin_rad = 0.0f;
            angle_begin_rad = encoder->multi_round_angle_rad;
            constexpr float pp_search_angle_rad = 12.0f * PI;
            float motor_angle_rad = 0.0f;
            while(motor_angle_rad <= pp_search_angle_rad)
            {
                motor_angle_rad += 0.01f;
                foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                foc->elec_omega_rad_s = 0.01f * 1000.0f;
                sleep(1);
            }
            sleep(500);
            float angle_end_rad = 0.0f;
            angle_end_rad = encoder->multi_round_angle_rad;
            foc->Iqd_target = {0.0f, 0.0f}; // turn off the motor
            sleep(100);
            // Assumed that our sensor direction is calibrated, so it must have: angle_end_rad > angle_begin_rad. (for multi-round angle)
            float moved = std::fabsf(angle_end_rad - angle_begin_rad);
            if(moved < MIN_ANGLE_DETECT_MOVEMENT)
            {
                foc->DisarmWithError(MotorError::MOTOR_FAILED_TO_ROTATE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            // if(angle_end_rad < angle_begin_rad) angle_end_rad += PI2;
            int pole_pairs = (int)std::round((pp_search_angle_rad) / moved);
            if(pole_pairs <= 0 || pole_pairs >= 32)
            {
                foc->DisarmWithError(MotorError::POLE_PAIR_NUMBER_OUT_OF_RANGE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            foc->GetConfig().set_pole_pairs(pole_pairs);
            foc->GetConfig().set_pole_pairs_valid(true);
            if(foc->GetConfig().flux_linkage_valid())
            {
                if(!foc->GetConfig().torque_constant_valid())
                {
                    // flux = (2/3) * torque_constant / pole_pairs
                    foc->GetConfig().set_torque_constant((float)pole_pairs * foc->GetConfig().flux_linkage() * 1.5f);
                    foc->GetConfig().set_torque_constant_valid(true);
                }
                if(!foc->GetConfig().kv_rating_valid())
                {
                    // flux = 5.51328895422 / (pole_pairs * motor_kv)
                    foc->GetConfig().set_kv_rating((5.51328895422f / foc->GetConfig().flux_linkage()) / (float)pole_pairs);
                    foc->GetConfig().set_kv_rating_valid(true);
                }
            }
            stage = EstStage::NONE;
            stage_passed++;
            break;
        }
        case EstStage::SENSOR_ZERO_OFFSET_TESTING:
        {
            const auto encoder = foc->GetPrimaryEncoder().value();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_NOT_FOUND);
                foc->RemoveTaskByName(GetName());
                break;
            }
            foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
            foc->elec_angle_rad = 0.0f;
            sleep(1000); // move motor to elec angle 0
            // calculate error between given elec_angle_rad
            // with calculated encoder elec_angle_rad
            float total_offset = 0.0f;
            int sample_count = 0;
            // omega: 4pi rad/s, forward: 16pi rad, backward: 16pi rad, average: encoder_rad when: elec_angle_rad = 8PI
            // maybe spin for a whole round?
            // const float total_distance = PI2 * 8;
            const float total_distance = PI2 * foc->GetConfig().pole_pairs();
            const int total_ms = 1000 * (float)(total_distance / (2 * PI2));
            for(int i = 0; i <= total_ms; i++)
            {
                float angle_rad = total_distance * (float)i / (float)total_ms;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                // measured elec angle
                float meas_elec_angle = normalize_rad(encoder->single_round_angle_rad * foc->GetConfig().pole_pairs());

                // #1
                float offset = meas_elec_angle - foc->elec_angle_rad;
                if(offset >= PI2) offset -= PI2;
                else if(offset <= -PI2) offset += PI2;
                total_offset += offset;

                // #2
                // total_offset += normalize_rad(meas_elec_angle - foc->elec_angle_rad);

                // #3
                // total_offset += meas_elec_angle;

                sample_count++;
                sleep(1);
            }
            sleep(500);
            for(int i = total_ms; i >= 0; i--)
            {
                float angle_rad = total_distance * (float)i / (float)total_ms;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                float meas_elec_angle = normalize_rad(encoder->single_round_angle_rad * foc->GetConfig().pole_pairs());

                float offset = meas_elec_angle - foc->elec_angle_rad;
                if(offset >= PI2) offset -= PI2;
                else if(offset <= -PI2) offset += PI2;
                total_offset += offset;

                // total_offset += normalize_rad(meas_elec_angle - foc->elec_angle_rad);

                // total_offset += meas_elec_angle;

                sample_count++;
                sleep(1);
            }
            foc->Iqd_target = {0.0f, 0.0f}; // stop the motor
            total_offset /= (float)sample_count;
            total_offset = normalize_rad(total_offset);
            foc->GetConfig().set_sensor_zero_offset_rad(total_offset);
            foc->GetConfig().set_sensor_zero_offset_valid(true);
            stage = EstStage::NONE;
            stage_passed++;
            break;
        }
        case EstStage::SENSOR_CUSTOM_CALIBRATING:
        {
            const auto encoder = foc->GetPrimaryEncoder().value();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_NOT_FOUND);
                foc->RemoveTaskByName(GetName());
                break;
            }
            if(*encoder == "MT6835") // MT6835 Custom Calibration: See Datasheet 9.2, Client Self-Calibration
            {
                if(foc->GetConfig().sensor_direction_valid() &&
                    (foc->GetConfig().pole_pairs() > 0 && foc->GetConfig().pole_pairs_valid()))
                {
                    // cast the pointer to mt6835 instance
                    const auto mt6835 = reinterpret_cast<Encoder::EncoderMT6835*>(encoder);
                    if(mt6835->cal_gpio.has_value()) // we have a CAL_EN gpio pin set
                    {
                        if(Encoder::EncoderMT6835::SelfCalibState state{}; mt6835->GetSelfCalibrationState(state) == FuncRetCode::OK)
                        {
                            if(state == Encoder::EncoderMT6835::SelfCalibState::NO_CALIB)
                            {
                                constexpr float calib_speed_rpm = 300.0f;
                                constexpr float calib_max_accel_rpm_s = 12.0f;
                                constexpr float calib_max_decel_rpm_s = 20.0f;
                                constexpr float calib_total_distance_rev = 512.0f;
                                // Step #1, Write AUTOCAL_FREQ
                                if(const auto ret = mt6835->SetSelfCalibrationRPM(calib_speed_rpm); ret != FuncRetCode::OK)
                                {
                                    foc->DisarmWithError(MotorError::PRIMARY_SENSOR_CUSTOM_CALIBRATION_FAILED);
                                    foc->RemoveTaskByName(GetName());
                                    break;
                                }
                                // Step #2, Pre-locating, Uq = u, theta = 270 ~ Ud = u, theta = 0
                                foc->Iqd_target = {0.0f, 0.0f};
                                foc->elec_angle_rad = 0.0f;
                                sleep(500);
                                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                                sleep(500);
                                // foc->Iqd_target = {0.0f, 0.0f};
                                // sleep(200);
                                // Step #3, Plan a trajectory
                                // Requirements: 1) BASE speed = 150 RPM 2) Cruise distance > 64 rounds
                                // BASE RPM -> ELEC RAD/S
                                const float target_pos = REV2RAD(calib_total_distance_rev * foc->GetConfig().pole_pairs());
                                const float target_speed = RPM2RAD(calib_speed_rpm, foc->GetConfig().pole_pairs());
                                const float max_accel = RPM2RAD(calib_max_accel_rpm_s, foc->GetConfig().pole_pairs());
                                const float max_decel = RPM2RAD(calib_max_decel_rpm_s, foc->GetConfig().pole_pairs());
                                TrajController traj{};
                                // Unit: ELEC, Speed: RADS, Pos: RAD
                                traj.PlanTrajectory(target_pos, 0.0f, 0.0f, target_speed, max_accel, max_decel);
                                uint8_t temp_counter = 0;
                                bool is_set_pin_high = false;
                                while(!traj.GetState())
                                {
                                    traj.Update(0.005f);
                                    foc->elec_angle_rad = normalize_rad(traj.GetCurrPos());
                                    foc->elec_omega_rad_s = traj.GetCurrSpeed();
                                    if(traj.GetTrajStage() == TrajController::TrajStage::CRUISING)
                                    {
                                        if(temp_counter >= 10)
                                        {
                                            if(!is_set_pin_high)
                                            {
                                                // Set CAL_EN pin High
                                                mt6835->cal_gpio.value()->Set();
                                                is_set_pin_high = true;
                                            }
                                            else // continue to check the calibration state, if success, we can decelerate in advance.
                                            {
                                                mt6835->GetSelfCalibrationState(state);
                                                if(state == Encoder::EncoderMT6835::SelfCalibState::CALIB_FAILED ||
                                                    state == Encoder::EncoderMT6835::SelfCalibState::CALIB_SUCCESS)
                                                {
                                                    traj.DecelerateInAdvance();
                                                }
                                            }
                                            temp_counter = 0;
                                        }
                                        else temp_counter++;
                                    }
                                    sleep(5);
                                }
                                if(state != Encoder::EncoderMT6835::SelfCalibState::CALIB_SUCCESS) mt6835->GetSelfCalibrationState(state);
                                sleep(10);
                                mt6835->cal_gpio.value()->Clear();
                                if(state != Encoder::EncoderMT6835::SelfCalibState::CALIB_SUCCESS)
                                {
                                    foc->DisarmWithError(MotorError::PRIMARY_SENSOR_CUSTOM_CALIBRATION_FAILED);
                                    foc->RemoveTaskByName(GetName());
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            stage = EstStage::NONE;
            is_sensor_custom_calibrated = true;
            break;
        }
    default: sleep(100);
        break;
    }
}
}
