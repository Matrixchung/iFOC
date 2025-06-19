#include "foc_task_basic_param_calib.hpp"
#include "foc_curr_loop_pi.hpp"
#include <cfloat>

#define foc GetMotor<FOCMotor>()

static constexpr float G9 = 2.0f;
static constexpr size_t AVG_WINDOW_SIZE = 10000;

namespace iFOC::FOC
{

class InjectTask : public Task
{
public:
    explicit InjectTask(BasicParamCalibTask &_task) : Task("InjectTask"), parent(_task)
    {
        RegisterTask(TaskType::RT_TASK);
    }
    void UpdateRT(float Ts) final
    {
        if(parent.stage == BasicParamCalibTask::EstStage::FLUX_LINKAGE_TESTING)
        {
            foc->Uqd_target.q += parent.data.flux_est.harmonic_reg_9_output;
        }
    }
private:
    BasicParamCalibTask &parent;
};

BasicParamCalibTask::BasicParamCalibTask() : Task("BasicParam")
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.rtos_priority = configMAX_PRIORITIES - 3;
}

void BasicParamCalibTask::InitNormal()
{
    while(foc->GetTaskByName("TonePlayer")) sleep(500);
    sleep(500);
    foc->BypassTaskByName("WaveGen");
    while(!foc->GetCurrSense()->IsCalibrated()) sleep(100);
}

void BasicParamCalibTask::UpdateNormal()
{
    switch(stage)
    {
        case EstStage::NONE:
        {
            sleep(100);
            memset(&data, 0, sizeof(data));
            foc->Arm();
            sleep(10);
            if(!foc->GetConfig().phase_resistance_valid())
            {
                stage = EstStage::PHASE_RESISTANCE_START;
                break;
            }
            if(!foc->GetConfig().phase_inductance_valid())
            {
                stage = EstStage::PHASE_INDUCTANCE_START;
                break;
            }
            if(!foc->GetConfig().flux_linkage_valid())
            {
                stage = EstStage::FLUX_LINKAGE_START;
                break;
            }
            foc->Disarm();
            foc->state_machine.BackToLastState();
            foc->RemoveTaskByName(GetName());
            break;
        }
        case EstStage::PHASE_RESISTANCE_START:
        {
            foc->BypassTaskByName("WaveGen", "CurrLoop");
            foc->Arm();
            data.Rs_est.TARGET_LOOP_COUNT = BoardConfig.GetConfig().pwm_wave_freq() * 2; // 2s
            data.Rs_est.target_current = foc->GetConfig().calibration_current();
            foc->GetDriver()->DisableBridges(Bridge::HB_V, Bridge::LB_V);
            sleep(10);
            stage = EstStage::PHASE_RESISTANCE_TESTING_U;
            break;
        }
        case EstStage::PHASE_RESISTANCE_TESTED_U:
        {
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
            sleep(200);
            foc->Arm();
            sleep(200);
            foc->GetDriver()->DisableBridges(Bridge::HB_U, Bridge::LB_U);
            sleep(10);
            stage = EstStage::PHASE_RESISTANCE_TESTING_V;
            break;
        }
        case EstStage::PHASE_RESISTANCE_TESTED_V:
        {
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
            sleep(200); // let the remaining current flow
            stage = EstStage::PHASE_RESISTANCE_TESTING_W;
            break;
        }
        case EstStage::PHASE_RESISTANCE_TESTED_W:
        {
            foc->Disarm();
            sleep(200);
            float rs_result = 0.0f;
            float rs_max_result = -FLT_MAX;
            float rs_min_result = FLT_MAX;
            for(uint8_t i = 0; i < 3; i++)
            {
                data.Rs_est.Rs_result[i] = ABS(data.Rs_est.voltage_diff[i] * foc->GetBusSense()->voltage / data.Rs_est.target_current) * 0.5f;
                if(data.Rs_est.Rs_result[i] >= rs_max_result) rs_max_result = data.Rs_est.Rs_result[i];
                if(data.Rs_est.Rs_result[i] <= rs_min_result) rs_min_result = data.Rs_est.Rs_result[i];
                rs_result += data.Rs_est.Rs_result[i];
            }
            if(ABS(rs_max_result - rs_min_result) >= 0.5f)
            {
                foc->DisarmWithError(MotorError::PHASE_IMBALANCE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            else
            {
                rs_result *= 0.3333333333f;
                if(rs_result <= 0.001f || rs_result >= 50.0f)
                {
                    foc->DisarmWithError(MotorError::PHASE_RESISTANCE_OUT_OF_RANGE);
                    foc->RemoveTaskByName(GetName());
                    break;
                }
                else
                {
                    foc->GetConfig().set_phase_resistance(rs_result);
                    foc->GetConfig().set_phase_resistance_valid(true);
                }
            }
            stage = EstStage::NONE;
            break;
        }
        // Ls estimation @ 1KHz square wave
        // https://blog.csdn.net/linzhe_deep/article/details/118067983
        case EstStage::PHASE_INDUCTANCE_START:
        {
            data.Ls_est.TARGET_LOOP_COUNT = iFOC::BoardConfig.GetConfig().pwm_wave_freq(); // 1s
            data.Ls_est.test_voltage = 1.0f;
            data.Ls_est.pwm_freq_div_1k = iFOC::BoardConfig.GetConfig().pwm_wave_freq() / 1000;
            if(data.Ls_est.pwm_freq_div_1k <= 1)
            {
                foc->DisarmWithError(MotorError::PWM_WAVE_FREQUENCY_OUT_OF_RANGE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            // Disable Phase V and test phase U
            foc->GetDriver()->DisableBridges(Bridge::HB_V, Bridge::LB_V);
            sleep(10);
            stage = EstStage::PHASE_INDUCTANCE_TESTING_U;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTED_U:
        {
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
            sleep(250);
            // Enable Phase V
            foc->GetDriver()->EnableBridges(Bridge::HB_V, Bridge::LB_V);
            sleep(250); // let the remaining current flow
            // Disable Phase U
            foc->GetDriver()->DisableBridges(Bridge::HB_U, Bridge::LB_U);
            sleep(10);
            stage = EstStage::PHASE_INDUCTANCE_TESTING_V;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTED_V:
        {
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
            sleep(500); // let the remaining current flow
            stage = EstStage::PHASE_INDUCTANCE_TESTING_W;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTED_W:
        {
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
            sleep(250);
            foc->GetDriver()->EnableBridges(Bridge::HB_U, Bridge::LB_U,
                                            Bridge::HB_V, Bridge::LB_V,
                                            Bridge::HB_W, Bridge::LB_W);
            sleep(250);
            float ls_result = 0.0f;
            float ls_max_result = -FLT_MAX;
            float ls_min_result = FLT_MAX;
            for(const float i : data.Ls_est.Ls_result)
            {
                if(i >= ls_max_result) ls_max_result = i;
                if(i <= ls_min_result) ls_min_result = i;
                ls_result += i;
            }
            ls_result *= 0.3333333333f;
            if(ls_result >= 1.0f)
            {
                foc->DisarmWithError(MotorError::PHASE_INDUCTANCE_OUT_OF_RANGE);
                foc->RemoveTaskByName(GetName());
                break;
            }
            else
            {
                foc->GetConfig().set_q_axis_inductance(ls_max_result);
                foc->GetConfig().set_d_axis_inductance(ls_min_result);
                foc->GetConfig().set_phase_inductance(ls_result);
                foc->GetConfig().set_phase_inductance_valid(true);
            }
            stage = EstStage::NONE;
            break;
        }
        case EstStage::FLUX_LINKAGE_START:
        {
            memset(&data, 0, sizeof(data));
            data.flux_est.iqf_filter = Filter::SlidingFilter(AVG_WINDOW_SIZE);
            data.flux_est.idf_filter = Filter::SlidingFilter(AVG_WINDOW_SIZE);
            data.flux_est.uqf_star_filter = Filter::SlidingFilter(AVG_WINDOW_SIZE);
            data.flux_est.udf_star_filter = Filter::SlidingFilter(AVG_WINDOW_SIZE);
            data.flux_est.flux_result_filter = Filter::LowpassFilter(10.0f);
            foc->RemoveTaskByName("CurrLoop");
            foc->BypassTaskByName("EncArbiter");
            foc->UnbypassTaskByName("WaveGen");
            foc->elec_angle_rad = 0.0f;
            foc->Arm();
            foc->InsertTaskBeforeName("WaveGen", new CurrLoopPI);
            foc->InsertTaskBeforeName("WaveGen", new InjectTask(*this));
            foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
            sleep(500);
            foc->Iqd_target= {0.0f, 0.0f};
            sleep(200);
            // Spin 512*PI2 rad, cruise speed: 96pi rad/s (we = 300rad/s),
            // max_accel: 16pi rad/s^2, max_decel: 32pi rad/s^2
            data.flux_est.cruise_we = 96.0f * PI;
            data.flux_est.traj.PlanTrajectory(512.0f * PI2,0.0f,
                                0.0f,data.flux_est.cruise_we,
                                16.0f * PI,32.0f * PI);
            // data.flux_est.wave = WaveInjector(WaveInjector::WaveType::SINUSOIDAL);
            // data.flux_est.wave.SetFrequency(data.flux_est.cruise_we * 9.0f);
            float w0_9 = 9.0f * data.flux_est.cruise_we; // Inject frequency: 9*we
            float phase_comp_9 = w0_9 * G9 * iFOC::RT_LOOP_TS;
            float wc = 0.015f * data.flux_est.cruise_we;
            wc = _constrain(wc, 2.0f, 20.0f);
            // 9-Harmonic
            float w0_9_Ts = w0_9 * iFOC::RT_LOOP_TS;
            HAL::sinf_cosf_impl(w0_9_Ts, data.flux_est.sin_w0_9_Ts, data.flux_est.cos_w0_9_Ts);
            HAL::sinf_cosf_impl(phase_comp_9, data.flux_est.sin_comp_9, data.flux_est.cos_comp_9);
            data.flux_est.harmonic_reg.Reset();
            data.flux_est.harmonic_reg.wc = wc;
            // data.flux_est.harmonic_reg.max_step_current = foc->GetConfig().calibration_current();
            data.flux_est.harmonic_reg.output_limit = MIN(foc->GetConfig().max_voltage(), foc->GetBusSense()->voltage) * divSQRT_3;
            data.flux_est.harmonic_reg_9_output = 0.0f;
            wave.SetFrequency(w0_9 / PI2); // NOTE: rad/s to Hz!!
            wave.PrepareTable(iFOC::RT_LOOP_TS);
            stage = EstStage::FLUX_LINKAGE_TESTING;
            break;
        }
        case EstStage::FLUX_LINKAGE_TESTED:
        {
            foc->Disarm();
            foc->RemoveTaskByName("InjectTask");
            foc->RemoveTaskByName("CurrLoop");
            foc->state_machine.BackToLastState();
            foc->RemoveTaskByName(GetName());
            break;
        }
        default: sleep(100); break;
    }
}

void BasicParamCalibTask::UpdateRT(float Ts)
{
    switch(stage)
    {
        case EstStage::PHASE_RESISTANCE_TESTING_U:
        {
            if(data.Rs_est.loop_count >= data.Rs_est.TARGET_LOOP_COUNT)
            {
                data.Rs_est.loop_count = 0;
                stage = EstStage::PHASE_RESISTANCE_TESTED_U;
                break;
            }
            data.Rs_est.voltage_diff[0] += data.Rs_est.Ki * Ts * (data.Rs_est.target_current -
                                                                  foc->GetCurrSense()->shunt_values[0]);
            foc->GetDriver()->SetOutput3CHPu(0.5f + data.Rs_est.voltage_diff[0], 0.0f, 0.5f);
            data.Rs_est.loop_count++;
            break;
        }
        case EstStage::PHASE_RESISTANCE_TESTING_V:
        {
            if(data.Rs_est.loop_count >= data.Rs_est.TARGET_LOOP_COUNT)
            {
                data.Rs_est.loop_count = 0;
                stage = EstStage::PHASE_RESISTANCE_TESTED_V;
                break;
            }
            data.Rs_est.voltage_diff[1] += data.Rs_est.Ki * Ts * (data.Rs_est.target_current -
                                                                  foc->GetCurrSense()->shunt_values[1]);
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.5f + data.Rs_est.voltage_diff[1], 0.5f);
            data.Rs_est.loop_count++;
            break;
        }
        case EstStage::PHASE_RESISTANCE_TESTING_W:
        {
            if(data.Rs_est.loop_count >= data.Rs_est.TARGET_LOOP_COUNT)
            {
                data.Rs_est.loop_count = 0;
                stage = EstStage::PHASE_RESISTANCE_TESTED_W;
                break;
            }
            data.Rs_est.voltage_diff[2] += data.Rs_est.Ki * Ts * (data.Rs_est.target_current -
                                                                  foc->GetCurrSense()->shunt_values[2]);
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.5f, 0.5f + data.Rs_est.voltage_diff[2]);
            data.Rs_est.loop_count++;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTING_U:
        {
            if(data.Ls_est.loop_count >= data.Ls_est.TARGET_LOOP_COUNT)
            {
                foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
                float dI_by_dt = ABS(data.Ls_est.currents[0][1] - data.Ls_est.currents[0][0]) / (float)(Ts * (float)data.Ls_est.TARGET_LOOP_COUNT);
                data.Ls_est.Ls_result[0] = (data.Ls_est.test_voltage / dI_by_dt) * 0.5f;
                data.Ls_est.loop_count = 0;
                stage = EstStage::PHASE_INDUCTANCE_TESTED_U;
                break;
            }
            // Test inductance @ 1KHz frequency
            auto i = data.Ls_est.loop_count % data.Ls_est.pwm_freq_div_1k;
            uint8_t half = data.Ls_est.pwm_freq_div_1k / 2;
            data.Ls_est.currents[0][i < half] += foc->GetCurrSense()->shunt_values[0];
            float test_voltage_pu = (i < half) ? -data.Ls_est.test_voltage / foc->GetBusSense()->voltage :
                                    data.Ls_est.test_voltage / foc->GetBusSense()->voltage;
            foc->GetDriver()->SetOutput3CHPu(0.5f + test_voltage_pu, 0.0f, 0.5f);
            data.Ls_est.loop_count++;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTING_V:
        {
            if(data.Ls_est.loop_count >= data.Ls_est.TARGET_LOOP_COUNT)
            {
                foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
                float dI_by_dt = ABS(data.Ls_est.currents[1][1] - data.Ls_est.currents[1][0]) / (float)(Ts * (float)data.Ls_est.TARGET_LOOP_COUNT);
                data.Ls_est.Ls_result[1] = (data.Ls_est.test_voltage / dI_by_dt) * 0.5f;
                data.Ls_est.loop_count = 0;
                stage = EstStage::PHASE_INDUCTANCE_TESTED_V;
                break;
            }
            // Test inductance @ 1KHz frequency
            auto i = data.Ls_est.loop_count % data.Ls_est.pwm_freq_div_1k;
            uint8_t half = data.Ls_est.pwm_freq_div_1k / 2;
            data.Ls_est.currents[1][i < half] += foc->GetCurrSense()->shunt_values[1];
            float test_voltage_pu = (i < half) ? -data.Ls_est.test_voltage / foc->GetBusSense()->voltage :
                                    data.Ls_est.test_voltage / foc->GetBusSense()->voltage;
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.5f + test_voltage_pu, 0.5f);
            data.Ls_est.loop_count++;
            break;
        }
        case EstStage::PHASE_INDUCTANCE_TESTING_W:
        {
            if(data.Ls_est.loop_count >= data.Ls_est.TARGET_LOOP_COUNT)
            {
                foc->GetDriver()->SetOutput3CHPu(0.0f, 0.0f, 0.0f);
                float dI_by_dt = ABS(data.Ls_est.currents[2][1] - data.Ls_est.currents[2][0]) / (float)(Ts * (float)data.Ls_est.TARGET_LOOP_COUNT);
                data.Ls_est.Ls_result[2] = (data.Ls_est.test_voltage / dI_by_dt) * 0.5f;
                data.Ls_est.loop_count = 0;
                stage = EstStage::PHASE_INDUCTANCE_TESTED_W;
                break;
            }
            // Test inductance @ 1KHz frequency
            auto i = data.Ls_est.loop_count % data.Ls_est.pwm_freq_div_1k;
            uint8_t half = data.Ls_est.pwm_freq_div_1k / 2;
            data.Ls_est.currents[2][i < half] += foc->GetCurrSense()->shunt_values[2];
            float test_voltage_pu = (i < half) ? -data.Ls_est.test_voltage / foc->GetBusSense()->voltage :
                                    data.Ls_est.test_voltage / foc->GetBusSense()->voltage;
            foc->GetDriver()->SetOutput3CHPu(0.0f, 0.5f, 0.5f + test_voltage_pu);
            data.Ls_est.loop_count++;
            break;
        }
        case EstStage::FLUX_LINKAGE_TESTING:
        {
            data.flux_est.traj.Update(Ts);
            switch(data.flux_est.traj.GetTrajStage())
            {
                case TrajController::TrajStage::ACCELERATING:
                case TrajController::TrajStage::DECELERATING:
                {
                    foc->Iqd_target = {0.0f, foc->GetConfig().calibration_current()};
                    foc->elec_angle_rad = normalize_rad(data.flux_est.traj.GetCurrPos());
                    foc->elec_omega_rad_s = data.flux_est.traj.GetCurrSpeed(); // used to suppress harmonic wave
                    data.flux_est.harmonic_reg.Reset();
                    data.flux_est.harmonic_reg_9_output = 0.0f;
                    break;
                }
                case TrajController::TrajStage::CRUISING:
                {
                    // do the injection here, Id = negative (cannot use Id = 0 control)
                    // due to electric angle is open-loop aligned, Iq here represents real Id.
                    // float inject_current = data.flux_est.wave.GetWaveform(Ts) * foc->GetConfig().calibration_current() * 0.25f;
                    float inject_current = wave.GetWaveform(Ts) * foc->GetConfig().calibration_current() * 0.5f;
                    foc->Iqd_target = {-foc->GetConfig().calibration_current() + inject_current,
                                       foc->GetConfig().calibration_current()};
                    float iq_error = foc->Iqd_target.q - foc->Iqd_measured.q;
                    data.flux_est.harmonic_reg_9_output = data.flux_est.harmonic_reg.GetOutputWithSinCos(iq_error,
                                                                                                         data.flux_est.w0_9,
                                                                                                         data.flux_est.sin_w0_9_Ts,
                                                                                                         data.flux_est.cos_w0_9_Ts,
                                                                                                         data.flux_est.sin_comp_9,
                                                                                                         data.flux_est.cos_comp_9,
                                                                                                         Ts);
                    auto& iqf = data.flux_est.iqf;
                    auto& idf = data.flux_est.idf;
                    auto& uqf = data.flux_est.uqf_star;
                    auto& udf = data.flux_est.udf_star;
                    const auto we = data.flux_est.cruise_we;
                    const auto ld = foc->GetConfig().d_axis_inductance();
                    const auto lq = foc->GetConfig().q_axis_inductance();
                    iqf = data.flux_est.iqf_filter.GetOutput(foc->Iqd_measured.d);
                    idf = data.flux_est.idf_filter.GetOutput(foc->Iqd_measured.q);
                    uqf = data.flux_est.uqf_star_filter.GetOutput(foc->Uqd_target.d);
                    udf = data.flux_est.udf_star_filter.GetOutput(foc->Uqd_target.q);
                    float flux_result = ((uqf * idf) - (udf * iqf) - (we * (ld * idf * idf) + (lq * iqf * iqf))) / (idf * we);
                    flux_result = data.flux_est.flux_result_filter.GetOutput(flux_result, Ts);
                    foc->GetConfig().set_flux_linkage(flux_result);
                    foc->elec_angle_rad = normalize_rad(data.flux_est.traj.GetCurrPos());
                    foc->elec_omega_rad_s = data.flux_est.traj.GetCurrSpeed();
                    break;
                }
                default:
                {
                    foc->Iqd_target = {0.0f, 0.0f};
                    foc->elec_omega_rad_s = 0.0f;
                    if(foc->GetConfig().flux_linkage() > 0.0f && foc->GetConfig().flux_linkage() < 1.0f)
                    {
                        foc->GetConfig().set_flux_linkage_valid(true);
                    }
                    stage = EstStage::FLUX_LINKAGE_TESTED;
                    break;
                }
            }
            break;
        }
        default: break;
    }
}

}
