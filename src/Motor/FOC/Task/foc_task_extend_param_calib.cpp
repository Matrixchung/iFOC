#include "foc_task_extend_param_calib.hpp"
#include "../Controller/foc_curr_loop_pi.hpp"

#include "../../../Encoder/encoder_off_axis_base.hpp"

#define PEAK_FINDING_PHASE_OMEGA_RADS (40.0f)
#define LUT_CALIBRATION_PHASE_OMEGA_RADS (20.0f)

namespace iFOC::FOC
{
ExtendParamCalibTask::ExtendParamCalibTask() : Task("ExtCalib")
{
    RegisterTask(TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 5;
    config.stack_depth = 256;
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

void ExtendParamCalibTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    switch(stage)
    {
        case EstStage::NONE:
        {
            foc->Iqd_target = {0.0f, 0.0f};
            if(const auto enc = foc->GetEncoderByName("EncOffAxis"); enc && foc->GetPrimaryEncoder())
            {
                if(!((Encoder::EncoderOffAxisBase*)enc)->IsPeakCalibrated())
                {
                    foc->Arm();
                    stage = EstStage::ENCODER_OFF_AXIS_PEAK_FINDING;
                    break;
                }
                if(!((Encoder::EncoderOffAxisBase*)enc)->IsLUTCalibrated())
                {
                    foc->Arm();
                    stage = EstStage::ENCODER_OFF_AXIS_LUT_CALIBRATING;
                    break;
                }
            }
            foc->Disarm();
            foc->RemoveTaskByName("CurrLoop");
            foc->UnbypassTaskByName("EncArbiter");
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
        default:
        {
            sleep(100);
            break;
        }
    }
}
}
