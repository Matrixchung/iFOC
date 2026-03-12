#include "foc_task_extend_param_calib.hpp"
#include "../Controller/foc_curr_loop_pi.hpp"
#include "../Controller/foc_speed_loop_pi.hpp"

#include "../../../Encoder/encoder_off_axis_base.hpp"

#define PEAK_FINDING_PHASE_OMEGA_RADS (40.0f)
#define LUT_CALIBRATION_PHASE_OMEGA_RADS (20.0f)

namespace iFOC::FOC
{
constexpr size_t ANTICOGGING_LUT_SEGMENTS       = ANTICOGGING_LUT_POINTS - 1;
constexpr float ANTICOGGING_STEP_RAD            = PI2 / (float)ANTICOGGING_LUT_SEGMENTS;
// constexpr float ANTICOGGING_POS_ERR_TH_RAD      = ANTICOGGING_STEP_RAD * 0.5f;
// constexpr float ANTICOGGING_VEL_TH_RAD_S        = ANTICOGGING_STEP_RAD * 20.0f;
constexpr float ANTICOGGING_STABLE_HOLD_S       = 0.01f;
constexpr uint16_t ANTICOGGING_AVG_SAMPLES      = 8;
// If insufficient samples were collected for a single step during the following time,
// a motor error will be raised (ANTICOGGING_POS_UNSTABLE) and the process will be stopped.
constexpr float ANTICOGGING_MAX_TIME_PER_STEP_S = 10.0f;

ExtendParamCalibTask::ExtendParamCalibTask() : Task("ExtCalib")
{
    RegisterTask(TaskType::NORMAL_TASK, TaskType::MID_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 5;
    config.stack_depth = 512;
}

ExtendParamCalibTask::~ExtendParamCalibTask()
{
    const auto foc = GetMotor<FOCMotor>();
    if(anticogging_cw_map)
    {
        vPortFree(anticogging_cw_map);
        anticogging_cw_map = nullptr;
    }
    if(anticogging_ccw_map)
    {
        vPortFree(anticogging_ccw_map);
        anticogging_ccw_map = nullptr;
    }
    foc->state_machine.RequestState(MotorState::IDLE);
}

void ExtendParamCalibTask::InitNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    const auto& enc = foc->GetPrimaryEncoder();
    if(!enc)
    {
        foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
        foc->RemoveTaskByName(GetName());
        return;
    }
    if(!enc->IsResultValid())
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
    // foc->Arm();
    // sleep(100);
    // Because we've finished basic parameter calibration,
    // we are going to add current loop here
    foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
    // Pre-locating, Uq = u, theta = 270 ~ Ud = u, theta = 0
    // foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
    // sleep(500);
    // foc->Iqd_target = {0.0f, 0.0f};
    // sleep(200);
    stage = EstStage::NONE;
}

void ExtendParamCalibTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    switch(stage)
    {
        case EstStage::NONE:
        {
            if(const auto enc = foc->GetEncoderByName("EncOffAxis"); enc && foc->GetPrimaryEncoder())
            {
                if(!((Encoder::EncoderOffAxisBase*)enc)->IsPeakCalibrated())
                {
                    foc->BypassTaskByName("EncArbiter", "SpeedLoop");
                    foc->elec_angle_rad = 0.0f;
                    foc->elec_omega_rad_s = 0.0f;
                    foc->Uqd_target = {0.0f, 0.0f};
                    foc->Iqd_target = {0.0f, 0.0f};
                    foc->Arm();
                    stage = EstStage::ENCODER_OFF_AXIS_PEAK_FINDING;
                    break;
                }
                if(!((Encoder::EncoderOffAxisBase*)enc)->IsLUTCalibrated())
                {
                    foc->BypassTaskByName("EncArbiter", "SpeedLoop");
                    foc->elec_angle_rad = 0.0f;
                    foc->elec_omega_rad_s = 0.0f;
                    foc->Uqd_target = {0.0f, 0.0f};
                    foc->Iqd_target = {0.0f, 0.0f};
                    foc->Arm();
                    stage = EstStage::ENCODER_OFF_AXIS_LUT_CALIBRATING;
                    break;
                }
            }
            if(const auto enc = foc->GetPrimaryEncoder();
                enc && enc->GetEncoderType() == Encoder::Type::ABSOLUTE_ENCODER &&
                foc->anticogging_lut.getTableSize() != ANTICOGGING_LUT_POINTS &&
                foc->GetConfig().vel_kp() > 0.0f &&
                foc->GetConfig().vel_ki() > 0.0f &&
                foc->GetConfig().pos_kp() > 0.0f &&
                foc->GetConfig().anticogging_base_pos_err_deg() > 0.0f &&
                foc->GetConfig().anticogging_base_vel_err_rpm() > 0.0f &&
                foc->GetError() == 0) // a correct-tuned position loop is required for anticogging calibration
            {
                stage = EstStage::ANTICOGGING_INIT;
                break;
            }
            if(anticogging_cw_map)
            {
                vPortFree(anticogging_cw_map);
                anticogging_cw_map = nullptr;
            }
            if(anticogging_ccw_map)
            {
                vPortFree(anticogging_ccw_map);
                anticogging_ccw_map = nullptr;
            }
            foc->Disarm();
            // foc->RemoveTaskByName("CurrLoop");
            // foc->RemoveTaskByName("SpeedLoop");
            // foc->UnbypassTaskByName("EncArbiter");
            foc->state_machine.BackToLastState();
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::ENCODER_OFF_AXIS_PEAK_FINDING:
        {
            auto* encoder = foc->GetEncoderByName("EncOffAxis");
            if(encoder)
            {
                auto* off_axis = (Encoder::EncoderOffAxisBase*)encoder;
                if(foc->GetConfig().pole_pairs() > 0 && foc->GetConfig().pole_pairs_valid() && foc->GetConfig().deduction_ratio() > 1.0f)
                {
                    off_axis->SetCalibrationMode(Encoder::EncoderOffAxisBase::CalibrationState::PEAK);
                    foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                    foc->elec_angle_rad = 0.0f;
                    sleep(800); // move motor to elec angle 0
                    // spin output shaft for: forward 1.5 round, backward 1.5 round
                    const float spin_angle_rad = foc->GetConfig().pole_pairs() * PI2 * 1.5f * foc->GetConfig().deduction_ratio();
                    float motor_angle_rad = 0.0f;
                    while(motor_angle_rad <= spin_angle_rad)
                    {
                        motor_angle_rad += PEAK_FINDING_PHASE_OMEGA_RADS * 0.001f;
                        foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                        foc->elec_omega_rad_s = PEAK_FINDING_PHASE_OMEGA_RADS;
                        sleep(1);
                    }
                    sleep(50);
                    while(motor_angle_rad >= 0.0f)
                    {
                        motor_angle_rad -= PEAK_FINDING_PHASE_OMEGA_RADS * 0.001f;
                        foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                        foc->elec_omega_rad_s = -PEAK_FINDING_PHASE_OMEGA_RADS;
                        sleep(1);
                    }
                    sleep(50);
                    foc->Iqd_target = {0.0f, 0.0f};
                    foc->elec_angle_rad = 0.0f;
                    off_axis->SetCalibrationMode(Encoder::EncoderOffAxisBase::CalibrationState::NONE);
                    stage = EstStage::ENCODER_OFF_AXIS_DIRECTION_TESTING;
                    stage_passed++;
                    break;
                }
            }
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::ENCODER_OFF_AXIS_DIRECTION_TESTING:
        {
            auto* encoder = foc->GetEncoderByName("EncOffAxis");
            if(encoder)
            {
                auto* off_axis = (Encoder::EncoderOffAxisBase*)encoder;
                if(foc->GetConfig().pole_pairs() > 0 && foc->GetConfig().pole_pairs_valid() && foc->GetConfig().deduction_ratio() > 1.0f)
                {
                    if(off_axis->IsPeakCalibrated()) // peak calibrated -> raw angle valid
                    {
                        encoder->SetSign(1); // positive first
                        foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                        foc->elec_angle_rad = 0.0f;
                        sleep(800); // move motor to elec angle 0
                        // spin output shaft for: forward 0.1 round, backward 0.1 round
                        const float spin_angle_rad = foc->GetConfig().pole_pairs() * PI2 * 0.1f * foc->GetConfig().deduction_ratio();
                        float motor_angle_rad = 0.0f;
                        while(motor_angle_rad <= spin_angle_rad)
                        {
                            motor_angle_rad += PEAK_FINDING_PHASE_OMEGA_RADS * 0.001f;
                            foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                            foc->elec_omega_rad_s = PEAK_FINDING_PHASE_OMEGA_RADS;
                            sleep(1);
                        }
                        sleep(200);
                        const float mid_angle = encoder->raw_single_round_angle_rad;
                        while(motor_angle_rad >= 0.0f)
                        {
                            motor_angle_rad -= PEAK_FINDING_PHASE_OMEGA_RADS * 0.001f;
                            foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                            foc->elec_omega_rad_s = -PEAK_FINDING_PHASE_OMEGA_RADS;
                            sleep(1);
                        }
                        sleep(200);
                        foc->Iqd_target = {0.0f, 0.0f};
                        foc->elec_angle_rad = 0.0f;
                        const float end_angle = encoder->raw_single_round_angle_rad;
                        if(mid_angle < end_angle)
                        {
                            encoder->SetSign(-1);
                        }
                        off_axis->SaveConfig(foc->GetInternalID());
                        stage = EstStage::ENCODER_OFF_AXIS_LUT_CALIBRATING;
                        stage_passed++;
                        break;
                    }
                    stage = EstStage::NONE;
                    break;
                }
            }
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::ENCODER_OFF_AXIS_LUT_CALIBRATING:
        {
            auto* off_axis_base = foc->GetEncoderByName("EncOffAxis");
            auto* primary_encoder = foc->GetPrimaryEncoder();
            if(off_axis_base && primary_encoder)
            {
                if(primary_encoder->IsResultValid())
                {
                    auto* off_axis = (Encoder::EncoderOffAxisBase*)off_axis_base;
                    if(foc->GetConfig().pole_pairs() > 0 && foc->GetConfig().pole_pairs_valid() && foc->GetConfig().deduction_ratio() > 1.0f)
                    {
                        if(off_axis->IsPeakCalibrated())
                        {
                            constexpr auto CAL_POINTS = Encoder::OFF_AXIS_LUT_POINTS;
                            constexpr float Ts = 0.005f;

                            // spin output shaft for: forward 1.0 round
                            const float spin_angle_rad = foc->GetConfig().pole_pairs() * PI2 * 1.0f * foc->GetConfig().deduction_ratio();
                            const float total_time = spin_angle_rad / (LUT_CALIBRATION_PHASE_OMEGA_RADS);
                            const float sample_dt = total_time / (float)CAL_POINTS;
                            float motor_angle_rad = 0.0f;
                            Vector<std::pair<float, float>> lut_samples{};
                            lut_samples.reserve(CAL_POINTS);

                            foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                            foc->elec_angle_rad = 0.0f;
                            sleep(800); // move motor to elec angle 0

                            const float primary_start = primary_encoder->multi_round_angle_rad;
                            float ap_raw_prev = off_axis->raw_single_round_angle_rad;
                            float ap_rel_accumulated = 0.0f;

                            uint16_t sample_index = 0;
                            float next_sample_time = 0.0f;
                            float elapsed_time = 0.0f;

                            while(sample_index < CAL_POINTS)
                            {
                                motor_angle_rad += LUT_CALIBRATION_PHASE_OMEGA_RADS * Ts;
                                elapsed_time += Ts;
                                foc->elec_angle_rad = normalize_rad(motor_angle_rad);
                                foc->elec_omega_rad_s = LUT_CALIBRATION_PHASE_OMEGA_RADS;
                                if(elapsed_time >= next_sample_time)
                                {
                                    const float Ai_rel = (primary_encoder->multi_round_angle_rad - primary_start) / (float)foc->GetConfig().deduction_ratio();
                                    const float ap_raw = off_axis->raw_single_round_angle_rad;
                                    const float delta_ap = normalize_rad_pm_pi(ap_raw - ap_raw_prev);
                                    ap_rel_accumulated += delta_ap;
                                    ap_raw_prev = ap_raw;
                                    const float error = normalize_rad_pm_pi(Ai_rel - ap_rel_accumulated);
                                    lut_samples.emplace_back(ap_raw, error);
                                    sample_index++;
                                    next_sample_time += sample_dt;
                                }
                                sleep(Ts * 1000.0f);
                            }

                            sleep(50);
                            foc->Iqd_target = {0.0f, 0.0f};
                            foc->elec_omega_rad_s = 0.0f;

                            float mean_error = 0.0f;
                            for(const auto& s : lut_samples) mean_error += s.second;
                            mean_error /= (float)lut_samples.size();

                            for(auto& s : lut_samples)
                            {
                                s.second = normalize_rad_pm_pi(s.second - mean_error);
                            }

                            off_axis->offset_lut = DataType::LookupTable(Encoder::OFF_AXIS_LUT_POINTS, 0.0f, PI2);
                            off_axis->offset_lut.fillFromSamplesPeriodic(lut_samples);
                            off_axis->SaveConfig(foc->GetInternalID());

                            foc->elec_angle_rad = 0.0f;

                            stage = EstStage::NONE;
                            stage_passed++;
                            break;
                        }
                        stage = EstStage::NONE;
                        break;
                    }
                }
            }
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::ANTICOGGING_INIT:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
                foc->RemoveTaskByName(GetName());
                break;
            }

            anticogging.index = 0;
            anticogging.stable_timer = 0.0f;
            anticogging.per_step_timer = 0.0f;
            anticogging.start_single_round_rad = 0.0f;
            anticogging.start_multi_round_rad = 0.0f;
            anticogging.iq_meas_acc = 0.0f;
            anticogging.iq_count = 0;
            anticogging.prev_anticogging_enabled = foc->GetConfig().enable_anticogging();
            foc->GetConfig().set_enable_anticogging(false);

            if(anticogging_cw_map)
            {
                vPortFree(anticogging_cw_map);
                anticogging_cw_map = nullptr;
            }
            if(anticogging_ccw_map)
            {
                vPortFree(anticogging_ccw_map);
                anticogging_ccw_map = nullptr;
            }

            anticogging_cw_map = (float*)pvPortMalloc(ANTICOGGING_LUT_SEGMENTS * sizeof(float));
            anticogging_ccw_map = (float*)pvPortMalloc(ANTICOGGING_LUT_SEGMENTS * sizeof(float));

            if(!anticogging_cw_map || !anticogging_ccw_map)
            {
                foc->DisarmWithError(MotorError::SYSTEM_MEM_ALLOCATION_FAILED);
                foc->RemoveTaskByName(GetName());
                break;
            }

            foc->UnbypassTaskByName("EncArbiter", "CurrLoop");
            if(!foc->GetTaskByName("SpeedLoop")) foc->InsertTaskBeforeName("CurrLoop", new SpeedLoopPI);
            foc->UnbypassTaskByName("SpeedLoop");
            sleep(100);
            auto current_target = foc->GetTargetMotionStruct(Motion::Ref::BASE, Motion::TorqueUnit::AMP, Motion::SpeedUnit::RADS, Motion::PosUnit::RAD);
            const auto current_motion = foc->GetCurrentMotionStruct(current_target);
            current_target.torque.value = 0.0f;
            current_target.speed.value = 0.0f;
            current_target.pos.value = current_motion.pos.value;
            foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
            foc->SetTargetMotion(current_target);
            foc->Arm();

            sleep(500); // wait for the initial state to steady
            anticogging.start_single_round_rad = encoder->compensated_single_round_angle_rad;
            anticogging.start_multi_round_rad = encoder->multi_round_angle_rad;

            stage = EstStage::ANTICOGGING_TESTING_CW;
            break;
        }
        case EstStage::ANTICOGGING_TESTED:
        {
            sleep(100);
            foc->Disarm();

            foc->anticogging_lut = DataType::LookupTable(ANTICOGGING_LUT_POINTS, 0.0f, PI2);
            const size_t lut_offset =
                (size_t)std::lroundf(normalize_rad(anticogging.start_single_round_rad) *
                             (float)ANTICOGGING_LUT_SEGMENTS * divPI2) % ANTICOGGING_LUT_SEGMENTS;
            constexpr int window = ANTICOGGING_AVG_SAMPLES / 2;
            for(size_t i = 0; i < ANTICOGGING_LUT_SEGMENTS; ++i)
            {
                float Icogging_filtered = 0.0f;
                {
                    double acc = 0.0;
                    int cnt = 0;
                    constexpr int half = window / 2;
                    for(int k = -half; k <= half; ++k)
                    {
                        int idx = i + k;
                        while(idx < 0) idx += (int)ANTICOGGING_LUT_SEGMENTS;
                        while(idx >= (int)ANTICOGGING_LUT_SEGMENTS) idx -= (int)ANTICOGGING_LUT_SEGMENTS;
                        const float Icogging = 0.5f * (anticogging_cw_map[idx] + anticogging_ccw_map[idx]);
                        acc += (double)Icogging;
                        ++cnt;
                    }
                    Icogging_filtered = (cnt > 0) ? (float)(acc / (double)cnt) : 0.0f;
                }
                const size_t lut_index = (lut_offset + i) % ANTICOGGING_LUT_SEGMENTS;
                foc->anticogging_lut.setValueByIndex(lut_index, Icogging_filtered);
            }
            foc->anticogging_lut.setValueByIndex(ANTICOGGING_LUT_SEGMENTS, foc->anticogging_lut.getValueByIndex(0));

            if(anticogging_cw_map)
            {
                vPortFree(anticogging_cw_map);
                anticogging_cw_map = nullptr;
            }
            if(anticogging_ccw_map)
            {
                vPortFree(anticogging_ccw_map);
                anticogging_ccw_map = nullptr;
            }

            char key[sizeof(ANTICOGGING_LUT_DB_KEY_PREFIX) + 1];
            memcpy(key, ANTICOGGING_LUT_DB_KEY_PREFIX, sizeof(ANTICOGGING_LUT_DB_KEY_PREFIX) - 1);
            key[sizeof(ANTICOGGING_LUT_DB_KEY_PREFIX) - 1] = foc->GetInternalID() + '0';
            key[sizeof(ANTICOGGING_LUT_DB_KEY_PREFIX)] = '\0';
            BlobNVMStorage().ClearNVM(key); // clear lut first

            auto lut_serialized_size = foc->anticogging_lut.getSerializedSize();
            uint8_t* serialize_buffer = (uint8_t*)pvPortMalloc(lut_serialized_size * sizeof(uint8_t));
            if(serialize_buffer)
            {
                if(foc->anticogging_lut.serialize(serialize_buffer, lut_serialized_size))
                {
                    BlobNVMStorage().SaveNVM(key, serialize_buffer, lut_serialized_size);
                }
                vPortFree(serialize_buffer);
                serialize_buffer = nullptr;
                foc->GetConfig().set_enable_anticogging(anticogging.prev_anticogging_enabled);
            }
            stage_passed++;
            stage = EstStage::NONE;
            break;
        }
        default:
        {
            sleep(100);
            break;
        }
    }
}

void ExtendParamCalibTask::UpdateMid(float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    const auto encoder = foc->GetPrimaryEncoder();
    if(!encoder)
    {
        stage = EstStage::NONE;
        return;
    }
    switch(stage)
    {
        case EstStage::ANTICOGGING_TESTING_CW:
        {
            if(!anticogging_cw_map || !anticogging_ccw_map)
            {
                stage = EstStage::NONE;
                break;
            }
            if(anticogging.index < (int32_t)ANTICOGGING_LUT_SEGMENTS)
            {
                anticogging.per_step_timer += Ts;
                if(anticogging.per_step_timer > ANTICOGGING_MAX_TIME_PER_STEP_S)
                {
                    foc->DisarmWithError(MotorError::ANTICOGGING_POS_UNSTABLE);
                    stage = EstStage::NONE;
                    break;
                }
                const float target_multi_round_rad = anticogging.start_multi_round_rad + (float)anticogging.index * ANTICOGGING_STEP_RAD;
                Motion target_motion
                {
                    .ref = Motion::Ref::BASE,
                    .torque = {},
                    .speed = {},
                    .pos = {target_multi_round_rad, 0.0f, Motion::PosUnit::RAD}
                };
                foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
                foc->SetTargetMotion(target_motion);
                const float pos_err = std::fabsf(target_multi_round_rad - encoder->multi_round_angle_rad);
                const float rad = DEG2RAD(foc->GetConfig().anticogging_base_pos_err_deg());
                const float rad_s = RPM2RAD(foc->GetConfig().anticogging_base_vel_err_rpm(), 1);
                const bool stable =
                    (pos_err < rad) &&
                    (std::fabsf(encoder->angular_speed_rad_s) < rad_s);
                if(stable)
                {
                    anticogging.stable_timer += Ts;
                    if(anticogging.stable_timer >= ANTICOGGING_STABLE_HOLD_S)
                    {
                        anticogging.iq_meas_acc += foc->Iqd_measured.q;
                        ++anticogging.iq_count;
                        if(anticogging.iq_count >= ANTICOGGING_AVG_SAMPLES)
                        {
                            anticogging_cw_map[anticogging.index] = anticogging.iq_meas_acc / (float)anticogging.iq_count;
                            ++anticogging.index;
                            anticogging.stable_timer = 0.0f;
                            anticogging.per_step_timer = 0.0f;
                            anticogging.iq_meas_acc = 0.0f;
                            anticogging.iq_count = 0;
                            break;
                        }
                    }
                }
                else
                {
                    anticogging.stable_timer = 0.0f;
                }
            }
            else // CW finished, start CCW
            {
                anticogging.index = (int32_t)ANTICOGGING_LUT_SEGMENTS - 1;
                anticogging.stable_timer = 0.0f;
                anticogging.per_step_timer = 0.0f;
                anticogging.iq_meas_acc = 0.0f;
                anticogging.iq_count = 0;
                stage = EstStage::ANTICOGGING_TESTING_CCW;
                break;
            }
            break;
        }
        case EstStage::ANTICOGGING_TESTING_CCW:
        {
            if(!anticogging_cw_map || !anticogging_ccw_map)
            {
                stage = EstStage::NONE;
                break;
            }
            if(anticogging.index >= 0)
            {
                anticogging.per_step_timer += Ts;
                if(anticogging.per_step_timer > ANTICOGGING_MAX_TIME_PER_STEP_S)
                {
                    foc->DisarmWithError(MotorError::ANTICOGGING_POS_UNSTABLE);
                    stage = EstStage::NONE;
                    break;
                }
                const float target_multi_round_rad = anticogging.start_multi_round_rad + (float)anticogging.index * ANTICOGGING_STEP_RAD;
                Motion target_motion
                {
                    .ref = Motion::Ref::BASE,
                    .torque = {},
                    .speed = {},
                    .pos = {target_multi_round_rad, 0.0f, Motion::PosUnit::RAD}
                };
                foc->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
                foc->SetTargetMotion(target_motion);
                const float pos_err = std::fabsf(target_multi_round_rad - encoder->multi_round_angle_rad);
                const float rad = DEG2RAD(foc->GetConfig().anticogging_base_pos_err_deg());
                const float rad_s = RPM2RAD(foc->GetConfig().anticogging_base_vel_err_rpm(), 1);
                const bool stable =
                    (pos_err < rad) &&
                    (std::fabsf(encoder->angular_speed_rad_s) < rad_s);
                if(stable)
                {
                    anticogging.stable_timer += Ts;
                    if(anticogging.stable_timer >= ANTICOGGING_STABLE_HOLD_S)
                    {
                        anticogging.iq_meas_acc += foc->Iqd_measured.q;
                        ++anticogging.iq_count;
                        if(anticogging.iq_count >= ANTICOGGING_AVG_SAMPLES)
                        {
                            anticogging_ccw_map[anticogging.index] = anticogging.iq_meas_acc / (float)anticogging.iq_count;
                            --anticogging.index;
                            anticogging.stable_timer = 0.0f;
                            anticogging.per_step_timer = 0.0f;
                            anticogging.iq_meas_acc = 0.0f;
                            anticogging.iq_count = 0;
                            break;
                        }
                    }
                }
                else
                {
                    anticogging.stable_timer = 0.0f;
                }
            }
            else
            {
                anticogging.index = 0;
                anticogging.stable_timer = 0.0f;
                anticogging.per_step_timer = 0.0f;
                anticogging.iq_meas_acc = 0.0f;
                anticogging.iq_count = 0;
                stage = EstStage::ANTICOGGING_TESTED;
                break;
            }
            break;
        }
        default: break;
    }
}
}
