#include "foc_task_update_sense.hpp"

static constexpr uint8_t CALIBRATION_TIMEOUT_MS = 50;
static constexpr uint8_t TEMP_SENSE_UPDATE_TICKS = 5;

namespace iFOC
{
UpdateSenseTask::UpdateSenseTask() : Task("SenseTask")
{
    RegisterTask(TaskType::RT_TASK, TaskType::NORMAL_TASK);
    config.stack_depth = 512;
    config.rtos_priority = configMAX_PRIORITIES - 3;
}

void UpdateSenseTask::UpdateRT(const float Ts)
{
    const auto foc = GetMotor<FOCMotor>();
    foc->GetCurrSense()->Update(Ts);
    // only enable leakage current detection after basic param calibration (Rs/Ld calibration will disconnect one of three phases)
    if(to_underlying(foc->GetCurrentState()) > to_underlying(MotorState::BASIC_PARAM_CALIBRATION))
    {
        // New method for detecting shunt loss & leakage current: https://blog.csdn.net/amonghappyer/article/details/136825518
        // this method works for both one shunt / two shunts / three shunts sampling applications.
        // ** The lowpass f_lp & sampling window vary with electric frequency elec_omega_rad_s
        // Step #1: detect electric freq
        const auto F_e = ABS(RADS2HZ(foc->elec_omega_rad_s));
        // Step #2: range freq, & detect sampling window
        if(F_e >= 1.0f) // only detect F_e >= 1Hz
        {
            // Sampling window count is based on Ts
            // max frequency: (1.0 / Ts)
            const auto F_e_max = 1.0f / Ts;
            if(F_e <= F_e_max * 0.5f) // nyquist sampling theory
            {
                if(shunt_imbal_det_interp_idx >= 1 &&
                    shunt_imbal_det_window > 0 &&
                    shunt_imbal_det_counter < shunt_imbal_det_window) // in detection process
                {
                    const auto Ia_abs = ABS(foc->GetCurrSense()->shunt_values[0]);
                    const auto Ib_abs = ABS(foc->GetCurrSense()->shunt_values[1]);
                    const auto Ic_abs = ABS(foc->GetCurrSense()->shunt_values[2]);
                    const auto Iabc_abs_sum = (Ia_abs + Ib_abs + Ic_abs) * 0.25f;
                    const auto Iabc_abs_sum_lp = Iabc_abs_sum_filter.GetOutput(Iabc_abs_sum, Ts);
                    if(shunt_imbal_det_counter % shunt_imbal_det_interp_idx == 0) // fixed-size max window
                    {
                        Ia_abs_filter.GetOutput(Ia_abs);
                        Ib_abs_filter.GetOutput(Ib_abs);
                        Ic_abs_filter.GetOutput(Ic_abs);
                        Iabc_abs_sum_window[shunt_imbal_det_counter / shunt_imbal_det_interp_idx] = Iabc_abs_sum_lp;
                        Iabc_abs_sum_total += Iabc_abs_sum_lp;
                    }
                    shunt_imbal_det_counter++;
                }
                // else // determine imbalance & update window
                if(shunt_imbal_det_interp_idx == 0 || shunt_imbal_det_window == 0 || shunt_imbal_det_counter >= shunt_imbal_det_window)
                {
                    auto moving_avg_window = shunt_imbal_det_window < SHUNT_IMBAL_SUM_WINDOW_SIZE ? shunt_imbal_det_window : SHUNT_IMBAL_SUM_WINDOW_SIZE;
                    if(moving_avg_window > 0 && shunt_imbal_det_interp_idx >= 1)
                    {
                        const auto Iabc_abs_sum_window_avg = Iabc_abs_sum_total / (float)(moving_avg_window);
                        if(Iabc_abs_sum_window_avg > 0.0f)
                        {
                            float Iabc_abs_diff_sum_avg = 0.0f;
                            for(size_t i = 0; i < moving_avg_window; i++)
                            {
                                Iabc_abs_diff_sum_avg += ABS(Iabc_abs_sum_window[i] - Iabc_abs_sum_window_avg);
                            }
                            Iabc_abs_diff_sum_avg /= (float)(moving_avg_window);
                            const auto pct = (Iabc_abs_diff_sum_avg / Iabc_abs_sum_window_avg);
                            if(pct >= 0.2f) // 20%
                            {
                                const float Ia_abs_avg = Ia_abs_filter.GetCurrent();
                                const float Ib_abs_avg = Ib_abs_filter.GetCurrent();
                                const float Ic_abs_avg = Ic_abs_filter.GetCurrent();
                                // shunt detection
                                if(Ia_abs_avg >= Ib_abs_avg)
                                {
                                    if(Ib_abs_avg >= Ic_abs_avg) // min: Ic, max: Ia
                                    {
                                        if(Ia_abs_avg * 0.25f > Ic_abs_avg)
                                            foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_W);
                                    }
                                    else
                                    {
                                        if(Ia_abs_avg >= Ic_abs_avg) // min: Ib, max: Ia
                                        {
                                            if(Ia_abs_avg * 0.25f > Ib_abs_avg)
                                                foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_V);
                                        }
                                        else // min: Ib, max: Ic
                                        {
                                            if(Ic_abs_avg * 0.25f > Ib_abs_avg)
                                                foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_V);
                                        }
                                    }
                                }
                                else
                                {
                                    if(Ia_abs_avg >= Ic_abs_avg) // min: Ic, max: Ib
                                    {
                                        if(Ib_abs_avg * 0.25f > Ic_abs_avg)
                                            foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_W);
                                    }
                                    else
                                    {
                                        if(Ib_abs_avg >= Ic_abs_avg) // min: Ia, max: Ib
                                        {
                                            if(Ib_abs_avg * 0.25f > Ia_abs_avg)
                                                foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_U);
                                        }
                                        else // min: Ia, max: Ic
                                        {
                                            if(Ic_abs_avg * 0.25f > Ia_abs_avg)
                                                foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE | MotorError::MOTOR_PHASE_IMBALANCE_U);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    shunt_imbal_det_window = F_e_max / F_e; // update window
                    // detect interpolation index. for Iabc_abs_sum_window, we only have a size of SHUNT_IMBAL_SUM_WINDOW_SIZE.
                    shunt_imbal_det_interp_idx = shunt_imbal_det_window / SHUNT_IMBAL_SUM_WINDOW_SIZE;
                    if(shunt_imbal_det_interp_idx < 1) shunt_imbal_det_interp_idx = 1;
                    shunt_imbal_det_window = shunt_imbal_det_interp_idx * SHUNT_IMBAL_SUM_WINDOW_SIZE;
                    moving_avg_window = shunt_imbal_det_window < SHUNT_IMBAL_SUM_WINDOW_SIZE ? shunt_imbal_det_window : SHUNT_IMBAL_SUM_WINDOW_SIZE;
                    Iabc_abs_sum_filter.Reset();
                    Iabc_abs_sum_filter.SetFc(0.5f * F_e); // nyquist freq
                    Ia_abs_filter.SetSize(moving_avg_window);
                    Ib_abs_filter.SetSize(moving_avg_window);
                    Ic_abs_filter.SetSize(moving_avg_window);
                    Iabc_abs_sum_total = 0.0f;
                    shunt_imbal_det_counter = 0;
                }
            }
            else
            {
                Iabc_abs_sum_filter.Reset();
                Ia_abs_filter.Reset();
                Ib_abs_filter.Reset();
                Ic_abs_filter.Reset();
                Iabc_abs_sum_total = 0.0f;
                shunt_imbal_det_interp_idx = 0;
                shunt_imbal_det_window = 0;
                shunt_imbal_det_counter = 0;
            }
        }
        else
        {
            Iabc_abs_sum_filter.Reset();
            Ia_abs_filter.Reset();
            Ib_abs_filter.Reset();
            Ic_abs_filter.Reset();
            Iabc_abs_sum_total = 0.0f;
            shunt_imbal_det_interp_idx = 0;
            shunt_imbal_det_window = 0;
            shunt_imbal_det_counter = 0;
        }

        // fallback to leakage current detection method
        // const real_t leakage_current = foc->GetCurrSense()->shunt_values[0] + foc->GetCurrSense()->shunt_values[1] + foc->GetCurrSense()->shunt_values[2];
        // if(ABS(leakage_current) >= foc->GetConfig().max_current() * 0.2f) // max leakage current = 20% max current
        // {
        //     foc->DisarmWithError(MotorError::MOTOR_PHASE_IMBALANCE); // phase current imbalance
        // }
    }
    else
    {
        Iabc_abs_sum_filter.Reset();
        Ia_abs_filter.Reset();
        Ib_abs_filter.Reset();
        Ic_abs_filter.Reset();
        Iabc_abs_sum_total = 0.0f;
        shunt_imbal_det_interp_idx = 0;
        shunt_imbal_det_window = 0;
        shunt_imbal_det_counter = 0;
    }
    foc->Ialphabeta_measured = FOC_Clark(foc->GetCurrSense()->shunt_values);
}

void UpdateSenseTask::UpdateNormal()
{
    const auto foc = GetMotor<FOCMotor>();
    auto* sense = foc->GetBusSense();
    if(foc->GetInternalID() == 0) // only update BusSense if is primary instance
    {
        const auto ret = sense->Update();
        if(ret != FuncRetCode::OK) foc->ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
        else foc->ClearError(MotorError::BUS_SENSE_RESULT_INVALID);
    }
    if(sense->voltage <= 0.0f)
    {
        foc->ThrowError(MotorError::BUS_SENSE_RESULT_INVALID);
    }
    else
    {
        foc->ClearError(MotorError::BUS_SENSE_RESULT_INVALID);
        if(sense->voltage > BoardConfig().GetConfig().bus_overvoltage_limit())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVERVOLTAGE);
        if(sense->voltage < BoardConfig().GetConfig().bus_undervoltage_limit())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_UNDERVOLTAGE);
        if(sense->current > BoardConfig().GetConfig().bus_max_positive_current())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_DRAIN_CURRENT);
        if(sense->current < BoardConfig().GetConfig().bus_max_negative_current())
            foc->DisarmWithError(MotorError::MOTOR_DC_BUS_OVER_RECHARGE_CURRENT);
    }
    if(!foc->GetCurrSense()->IsCalibrated())
    {
        if(calibration_timeout_ms < CALIBRATION_TIMEOUT_MS) calibration_timeout_ms += 10;
        else foc->DisarmWithError(MotorError::MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT);
    }
    else
    {
        calibration_timeout_ms = 0;
        foc->ClearError(MotorError::MOTOR_CURR_SENSE_CALIBRATION_TIMEOUT);
    }
    if(temperature_sense_tick++ >= TEMP_SENSE_UPDATE_TICKS)
    {
        if(const auto core = foc->GetCoreTempSense())
            core->Update();
        if(const auto mosfet = foc->GetMosfetTempSense())
            mosfet->Update();
        if(const auto motor = foc->GetMotorTempSense())
            motor->Update();
        temperature_sense_tick = 0;
    }
    if(const auto ind = foc->GetIndicator())
        ind->Update(foc->GetInternalID(), foc->GetError(), foc->GetCurrentState(), foc->GetControlMode());

    sleep(10);
}
}