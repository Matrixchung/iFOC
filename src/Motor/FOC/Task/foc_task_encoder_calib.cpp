#include "foc_task_encoder_calib.hpp"
#include "../Controller/foc_curr_loop_pi.hpp"
#include "../../../DataType/lookup_table.hpp"

#define SAMPLES_PER_POLE_PAIR (128U)
#define ZERO_EST_PHASE_OMEGA_RADS (10.0f)

namespace iFOC::FOC
{
EncoderCalibTask::EncoderCalibTask() : Task("EncCalib")
{
    RegisterTask(TaskType::NORMAL_TASK, TaskType::MID_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 5;
    config.stack_depth = 1024;
}

EncoderCalibTask::~EncoderCalibTask()
{
    const auto foc = GetMotor<FOCMotor>();
    if(temp_map)
    {
        vPortFree(temp_map);
        temp_map = nullptr;
    }
    foc->state_machine.RequestState(MotorState::IDLE);
}

static constexpr float MIN_ANGLE_DETECT_MOVEMENT = PI2 / 101.0f;

void EncoderCalibTask::InitNormal()
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

void EncoderCalibTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
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
                stage = EstStage::SENSOR_ZERO_OFFSET_TEST_START;
                break;
            }
            // if(!is_sensor_custom_calibrated && stage_passed > 0)
            // {
            //     foc->Arm();
            //     stage = EstStage::SENSOR_CUSTOM_CALIBRATING;
            //     break;
            // }
            foc->Disarm();
            foc->RemoveTaskByName("CurrLoop");
            foc->UnbypassTaskByName("EncArbiter");
            foc->state_machine.BackToLastState();
            if(temp_map)
            {
                vPortFree(temp_map);
                temp_map = nullptr;
            }
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::SENSOR_DIRECTION_TESTING:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
                foc->RemoveTaskByName(GetName());
                break;
            }
            encoder->SetSign(1); // first: positive sign
            float previous_angle = 0.0f;
            float forward_moved = 0.0f;
            bool previous_angle_valid = false;
            for(int i = 0; i < 500; i++)
            {
                float angle_rad = PI2 * (float)i / 500.0f;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                sleep(2);
                const float current_angle = encoder->raw_single_round_angle_rad;
                if(!previous_angle_valid)
                {
                    previous_angle = current_angle;
                    previous_angle_valid = true;
                    continue;
                }
                forward_moved += normalize_rad_pm_pi(current_angle - previous_angle);
                previous_angle = current_angle;
            }
            for(int i = 499; i >= 0; i--)
            {
                float angle_rad = PI2 * (float)i / 500.0f;
                foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                foc->elec_angle_rad = normalize_rad(angle_rad);
                sleep(2);
            }
            foc->Iqd_target = {0.0f, 0.0f};
            sleep(200);
            // Use the accumulated circular displacement from the forward sweep.
            // Comparing mid_angle and end_angle directly fails when the encoder
            // crosses the [0, 2pi) wrap boundary during the sweep.
            const float moved = std::fabsf(forward_moved);
            if(moved < MIN_ANGLE_DETECT_MOVEMENT)
            {
                foc->DisarmWithError(MotorError::MOTOR_FAILED_TO_ROTATE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            if(forward_moved < 0.0f)
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
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
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
                foc->DisarmWithError(MotorError::MOTOR_POLE_PAIR_NUMBER_OUT_OF_RANGE);
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
        case EstStage::SENSOR_ZERO_OFFSET_TEST_START:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
                foc->RemoveTaskByName(GetName());
                break;
            }
            if(!foc->GetConfig().pole_pairs_valid() || foc->GetConfig().pole_pairs() > 30)
            {
                foc->GetConfig().set_pole_pairs(0);
                foc->GetConfig().set_pole_pairs_valid(false);
                stage = EstStage::NONE;
                break;
            }

            char key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) + 1];
            memcpy(key, Encoder::NONLINEAR_LUT_DB_KEY_PREFIX, sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) - 1);
            key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) - 1] = foc->GetInternalID() + '0';
            key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX)] = '\0';
            BlobNVMStorage().ClearNVM(key); // clear lut first

            if(temp_map)
            {
                vPortFree(temp_map);
                temp_map = nullptr;
            }

            temp_map = (float*)pvPortMalloc(SAMPLES_PER_POLE_PAIR * foc->GetConfig().pole_pairs() * sizeof(float));
            if(!temp_map)
            {
                foc->DisarmWithError(MotorError::SYSTEM_MEM_ALLOCATION_FAILED);
                foc->RemoveTaskByName(GetName());
                break;
            }

            zero_est.sample_count = 0;
            zero_est.timer = 0.0f;
            zero_est.next_sample_time = 0.0f;
            zero_est.elec_angle_rad = 0.0;

            foc->GetConfig().set_sensor_zero_offset_rad(0.0f);
            foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
            foc->elec_angle_rad = 0.0f;
            sleep(1000); // move motor to elec angle 0

            stage = EstStage::SENSOR_ZERO_OFFSET_TESTING_CW;

            break;
        }
        case EstStage::SENSOR_ZERO_OFFSET_TESTED:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                foc->DisarmWithError(MotorError::PRIMARY_SENSOR_COMPONENT_MISSING);
                foc->RemoveTaskByName(GetName());
                break;
            }
            const uint32_t M  = foc->GetConfig().pole_pairs() * SAMPLES_PER_POLE_PAIR;
            double moving_average = 0.0;
            for(uint16_t i = 0; i < (uint16_t)M; i++)
            {
                // moving_average += (double)temp_map[i];
                moving_average += (double)normalize_rad_pm_pi(temp_map[i]);
            }
            moving_average /= ((double)SAMPLES_PER_POLE_PAIR);

            if(encoder->GetEncoderType() == Encoder::Type::ABSOLUTE_ENCODER)
            {
                const float zero_offset_mech = moving_average / (float)foc->GetConfig().pole_pairs();
                constexpr size_t LUT_SEGMENTS = Encoder::NONLINEAR_LUT_POINTS - 1;
                DataType::LookupTable lut(Encoder::NONLINEAR_LUT_POINTS, 0.0f, PI2);

                // calculate rotation offset
                // temp_map (based on 0-elec rad reference axis) -> lut (based on 0-sensor rad reference axis)
                const float raw0 = normalize_rad(temp_map[0]);
                const size_t lut_offset = (size_t)std::lroundf(raw0 * (float)LUT_SEGMENTS * divPI2) % LUT_SEGMENTS;

                // resampling (SAMPLES_PER_POLE_PAIR * pp -> NONLINEAR_LUT_POINTS) && FIR smoothing to LUT
                constexpr int window = (int)SAMPLES_PER_POLE_PAIR / 2U;
                for(size_t i = 0; i < LUT_SEGMENTS; i++)
                {
                    // LUT 第 i 个周期分段在原始样本序列中的中心位置
                    const int center = (int)((uint64_t)i * (uint64_t)M / (uint64_t)LUT_SEGMENTS);
                    double accmulate = 0.0;
                    int count = 0;
                    for(int j = -window / 2; j < window / 2; ++j)
                    {
                        int idx = center + j;
                        while(idx < 0) idx += (int)M;
                        while(idx >= (int)M) idx -= (int)M;
                        const float residual = normalize_rad_pm_pi(temp_map[(uint32_t)idx] - zero_offset_mech);
                        accmulate += residual;
                        ++count;
                    }
                    const float lut_value = (count > 0) ? (accmulate / (double)count) : 0.0f;
                    const size_t lut_index = (lut_offset + i) % LUT_SEGMENTS;
                    lut.setValueByIndex(lut_index, lut_value);
                }
                lut.setValueByIndex(LUT_SEGMENTS, lut.getValueByIndex(0)); // wrap around

                // Check LUT validity
                constexpr float MAX_NONLINEAR_LUT_PEAK_RAD = 0.1f;
                float lut_peak_abs_rad = 0.0f;
                bool lut_valid = true;

                for(size_t i = 0; i < LUT_SEGMENTS; ++i)
                {
                    const float value = lut.getValueByIndex(i);

                    if(!std::isfinite(value))
                    {
                        lut_valid = false;
                        break;
                    }

                    lut_peak_abs_rad = IFOC_MAX(
                        lut_peak_abs_rad,
                        std::fabsf(value)
                    );
                }

                if(!lut_valid || lut_peak_abs_rad > MAX_NONLINEAR_LUT_PEAK_RAD)
                {
                    foc->DisarmWithError(MotorError::PRIMARY_SENSOR_CALIBRATION_FAILED);
                    foc->GetConfig().set_sensor_direction_valid(false);
                    foc->GetConfig().set_sensor_zero_offset_rad(0.0f);
                    foc->GetConfig().set_sensor_zero_offset_valid(false);
                    foc->RemoveTaskByName(GetName());
                    break;
                }

                if(temp_map)
                {
                    vPortFree(temp_map);
                    temp_map = nullptr;
                }

                char key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) + 1];
                memcpy(key, Encoder::NONLINEAR_LUT_DB_KEY_PREFIX, sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) - 1);
                key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX) - 1] = foc->GetInternalID() + '0';
                key[sizeof(Encoder::NONLINEAR_LUT_DB_KEY_PREFIX)] = '\0';

                auto lut_serialized_size = lut.getSerializedSize();
                uint8_t* serialize_buffer = (uint8_t*)pvPortMalloc(lut_serialized_size * sizeof(uint8_t));
                if(serialize_buffer)
                {
                    if(lut.serialize(serialize_buffer, lut_serialized_size))
                    {
                        // By changing FDB_KVDB_CTRL_SET_SEC_SIZE using fdb_kvdb_control(), larger KV is allowed.
                        BlobNVMStorage().SaveNVM(key, serialize_buffer, lut_serialized_size);
                    }
                    vPortFree(serialize_buffer);
                    serialize_buffer = nullptr;
                }
            }

            if(temp_map)
            {
                vPortFree(temp_map);
                temp_map = nullptr;
            }

            foc->GetConfig().set_sensor_zero_offset_rad(normalize_rad(moving_average));
            foc->GetConfig().set_sensor_zero_offset_valid(true);

            stage = EstStage::NONE;
            stage_passed++;
            break;
        }
    default: sleep(100);
        break;
    }
}

void EncoderCalibTask::UpdateMid(float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    switch(stage)
    {
        case EstStage::SENSOR_ZERO_OFFSET_TESTING_CW:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                stage = EstStage::SENSOR_ZERO_OFFSET_TEST_START;
                break;
            }
            zero_est.timer += Ts;
            // total elec angle: PI2 * foc->GetConfig().pole_pairs()
            if(zero_est.sample_count < (int16_t)(foc->GetConfig().pole_pairs() * SAMPLES_PER_POLE_PAIR))
            {
                if(zero_est.timer > zero_est.next_sample_time)
                {
                    zero_est.next_sample_time += PI2 / ((float)SAMPLES_PER_POLE_PAIR * ZERO_EST_PHASE_OMEGA_RADS);
                    const float ref_enc_single_round_rad = zero_est.elec_angle_rad / (float)foc->GetConfig().pole_pairs(); // elec -> motor
                    float error = encoder->raw_single_round_angle_rad - ref_enc_single_round_rad;
                    error = normalize_rad(error);
                    temp_map[zero_est.sample_count] = error;
                    zero_est.sample_count++;
                }
                zero_est.elec_angle_rad += ZERO_EST_PHASE_OMEGA_RADS * Ts;
            }
            else
            {
                zero_est.elec_angle_rad -= ZERO_EST_PHASE_OMEGA_RADS * Ts;
                zero_est.timer = 0.0f;
                zero_est.sample_count--;
                zero_est.next_sample_time = 0.0f;
                stage = EstStage::SENSOR_ZERO_OFFSET_TESTING_CCW;
                break;
            }
            foc->elec_angle_rad = normalize_rad(zero_est.elec_angle_rad);
            foc->elec_omega_rad_s = ZERO_EST_PHASE_OMEGA_RADS;
            break;
        }
        case EstStage::SENSOR_ZERO_OFFSET_TESTING_CCW:
        {
            const auto encoder = foc->GetPrimaryEncoder();
            if(!encoder)
            {
                stage = EstStage::SENSOR_ZERO_OFFSET_TEST_START;
                break;
            }
            zero_est.timer += Ts;
            if(zero_est.sample_count >= 0)
            {
                if(zero_est.timer > zero_est.next_sample_time)
                {
                    zero_est.next_sample_time += PI2 / ((float)SAMPLES_PER_POLE_PAIR * ZERO_EST_PHASE_OMEGA_RADS);
                    const float ref_enc_single_round_rad = zero_est.elec_angle_rad / (float)foc->GetConfig().pole_pairs(); // elec -> motor
                    float error = encoder->raw_single_round_angle_rad - ref_enc_single_round_rad;
                    error = normalize_rad(error);
                    // temp_map[zero_est.sample_count] = (temp_map[zero_est.sample_count] + error) * 0.5f; // fixed: given a 0.01(CW) & (2PI - 0.01)(CCW) error
                    // circular mean
                    temp_map[zero_est.sample_count] = normalize_rad(temp_map[zero_est.sample_count] + 0.5f * normalize_rad_pm_pi(error - temp_map[zero_est.sample_count]));
                    zero_est.sample_count--;
                }
                zero_est.elec_angle_rad -= ZERO_EST_PHASE_OMEGA_RADS * Ts;
            }
            else
            {
                stage = EstStage::SENSOR_ZERO_OFFSET_TESTED;
                break;
            }
            foc->elec_angle_rad = normalize_rad(zero_est.elec_angle_rad);
            foc->elec_omega_rad_s = ZERO_EST_PHASE_OMEGA_RADS;
            break;
        }
        default: break;
    }
}
}
