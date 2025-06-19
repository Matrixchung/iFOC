#include "foc_curr_loop_pi.hpp"

#define foc GetMotor<FOCMotor>()

static constexpr float G6 = 2.0f;
static constexpr float G12 = 2.0f;

namespace iFOC::FOC
{
CurrLoopPI::CurrLoopPI() : CurrLoopBase(),
                          q_harmonic_reg_6(30.0f, 5.0f, 0.0f),
                          q_harmonic_reg_12(30.0f, 5.0f, 0.0f),
                          d_harmonic_reg_6(30.0f, 5.0f, 0.0f),
                          d_harmonic_reg_12(30.0f, 5.0f, 0.0f) {}

void CurrLoopPI::InitCurrLoop()
{
    q_pi.Kp = foc->GetConfig().current_loop_bandwidth() * 2.0f * PI * foc->GetConfig().q_axis_inductance();
    q_pi.Ki = foc->GetConfig().current_loop_bandwidth() * 2.0f * PI * foc->GetConfig().phase_resistance();
    q_pi.limit = foc->GetConfig().max_voltage() * divSQRT_3;
    d_pi.Kp = foc->GetConfig().current_loop_bandwidth() * 2.0f * PI * foc->GetConfig().d_axis_inductance();
    d_pi.Ki = q_pi.Ki;
    d_pi.limit = q_pi.limit;
    q_harmonic_reg_6.output_limit = q_harmonic_reg_12.output_limit = \
    d_harmonic_reg_6.output_limit = d_harmonic_reg_12.output_limit = q_pi.limit;
    q_harmonic_reg_6.max_step_current = q_harmonic_reg_12.max_step_current = \
    d_harmonic_reg_6.max_step_current = d_harmonic_reg_12.max_step_current = 1.0f;
    enable_flux_feedforward = foc->GetConfig().flux_linkage_valid();
}

void CurrLoopPI::ResetCurrLoop()
{
    q_pi.Reset();
    d_pi.Reset();
    q_harmonic_reg_6.Reset();
    q_harmonic_reg_12.Reset();
    d_harmonic_reg_6.Reset();
    d_harmonic_reg_12.Reset();
    foc->Uqd_target = {0.0f, 0.0f};
}

void CurrLoopPI::UpdateCurrLoop(float Ts)
{
    // Q-Axis
    float iq_error = foc->Iqd_target.q - foc->Iqd_measured.q;
    float Uq_total_feedforward = 0.0f;
    float curr_Udc = MIN(foc->GetConfig().max_voltage(), foc->GetBusSense()->voltage);
    q_pi.limit = curr_Udc * divSQRT_3;

    // D-Axis
    float id_error = foc->Iqd_target.d - foc->Iqd_measured.d;
    float Ud_total_feedforward = 0.0f;
    d_pi.limit = q_pi.limit;

    if(enable_harmonic_regulator)
    {
        // Harmonic Regulator Settings
        float w0_6 = 6.0f * foc->elec_omega_rad_s;
        float phase_comp_6 = w0_6 * G6 * Ts;
        float w0_12 = 12.0f * foc->elec_omega_rad_s;
        float phase_comp_12 = w0_12 * G12 * Ts;
        float wc = 0.015f * ABS(foc->elec_omega_rad_s);
        // if(wc <= 2.0f) wc = 2.0f;
        // else if(wc >= 20.0f) wc = 20.0f;
        wc = _constrain(wc, 2.0f, 20.0f);
        // 6-Harmonic
        float w0_6_Ts = w0_6 * Ts;
        float sin_w0_6_Ts, cos_w0_6_Ts;
        HAL::sinf_cosf_impl(w0_6_Ts, sin_w0_6_Ts, cos_w0_6_Ts);
        float sin_comp_6, cos_comp_6;
        HAL::sinf_cosf_impl(phase_comp_6, sin_comp_6, cos_comp_6);
        // 12-Harmonic
        float w0_12_Ts = w0_12 * Ts;
        float sin_w0_12_Ts, cos_w0_12_Ts;
        HAL::sinf_cosf_impl(w0_12_Ts, sin_w0_12_Ts, cos_w0_12_Ts);
        float sin_comp_12, cos_comp_12;
        HAL::sinf_cosf_impl(phase_comp_12, sin_comp_12, cos_comp_12);
        // Q-Axis Harmonic Regulator
        q_harmonic_reg_6.wc = wc;
        q_harmonic_reg_6.output_limit = q_pi.limit;
        q_harmonic_reg_12.wc = wc;
        q_harmonic_reg_12.output_limit = q_pi.limit;
        float uq_6_output = q_harmonic_reg_6.GetOutputWithSinCos(iq_error,
                                                                 w0_6,
                                                                 sin_w0_6_Ts,
                                                                 cos_w0_6_Ts,
                                                                 sin_comp_6,
                                                                 cos_comp_6,
                                                                 Ts);
        float uq_12_output = q_harmonic_reg_12.GetOutputWithSinCos(iq_error,
                                                                   w0_12,
                                                                   sin_w0_12_Ts,
                                                                   cos_w0_12_Ts,
                                                                   sin_comp_12,
                                                                   cos_comp_12,
                                                                   Ts);
        Uq_total_feedforward += uq_6_output + uq_12_output;
        // D-Axis Harmonic Regulator
        d_harmonic_reg_6.wc = wc;
        d_harmonic_reg_6.output_limit = q_pi.limit;
        d_harmonic_reg_12.wc = wc;
        d_harmonic_reg_12.output_limit = q_pi.limit;
        float ud_6_output = d_harmonic_reg_6.GetOutputWithSinCos(id_error,
                                                                 w0_6,
                                                                 sin_w0_6_Ts,
                                                                 cos_w0_6_Ts,
                                                                 sin_comp_6,
                                                                 cos_comp_6,
                                                                 Ts);
        float ud_12_output = d_harmonic_reg_12.GetOutputWithSinCos(id_error,
                                                                   w0_12,
                                                                   sin_w0_12_Ts,
                                                                   cos_w0_12_Ts,
                                                                   sin_comp_12,
                                                                   cos_comp_12,
                                                                   Ts);
        Ud_total_feedforward += ud_6_output + ud_12_output;
    }
    // Uq = Rs * Iq + We * Ld * Id + We * flux
    // Ud = Rs * Id - We * Lq * Iq
    if(enable_pi_feedforward)
    {
        if(enable_flux_feedforward) Uq_total_feedforward += foc->elec_omega_rad_s * (foc->GetConfig().d_axis_inductance() * foc->Iqd_measured.d + foc->GetConfig().flux_linkage());
        else Uq_total_feedforward += foc->elec_omega_rad_s * (foc->GetConfig().d_axis_inductance() * foc->Iqd_measured.d);
        Ud_total_feedforward -= foc->elec_omega_rad_s * (foc->GetConfig().q_axis_inductance() * foc->Iqd_measured.q);
    }
    foc->Uqd_target.q = q_pi.GetOutput(iq_error, Ts, Uq_total_feedforward);
    foc->Uqd_target.d = d_pi.GetOutput(id_error, Ts, Ud_total_feedforward);
}

}