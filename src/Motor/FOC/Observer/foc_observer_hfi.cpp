#include "foc_observer_hfi.hpp"

namespace iFOC::FOC
{
ObserverHFI::ObserverHFI() : Task("HFIMain"),
                             Ia_lpf(BoardConfig().GetConfig().get_pwm_wave_freq() / 20),
                             Ib_lpf(BoardConfig().GetConfig().get_pwm_wave_freq() / 20)
{
    RegisterTask(TaskType::RT_TASK);
    encoder = new EncoderHFI();
    injector = new InjectorHFI();
    wave_injector.SetFrequency(BoardConfig().GetConfig().get_pwm_wave_freq() / 2); // up to 0.5x fPWM
}

ObserverHFI::~ObserverHFI()
{
    const auto foc = GetMotor<FOCMotor>();
    if(foc->RemoveEncoderByName("HFIEnc") != FuncRetCode::OK)
    {
        if(encoder) vPortFree(encoder);
    }
    if(foc->RemoveTaskByName("HFIInj") != FuncRetCode::OK)
    {
        if(injector) vPortFree(injector);
    }
}

void ObserverHFI::InitRT()
{
    const auto foc = GetMotor<FOCMotor>();
    if(encoder && injector)
    {
        // pll.Kp = 600.0f;
        // pll.Ki = 800000.0f;
        pll.Kp = 1000.0f;
        pll.Ki = 2500000.0f;
        // RPM to eRPM to rad/s
        pll.omega_limit_rad_s = RPM2RAD(1000.0f * foc->GetConfig().deduction_ratio(), foc->GetConfig().pole_pairs());
        encoder->result_valid = true;
        foc->AppendEncoder(encoder);
        foc->SetPrimaryEncoderIndex(foc->GetEncoders().size() - 1);
        foc->InsertTaskBeforeName("WaveGen", injector);
    }
    else foc->RemoveTaskByName(GetName()); // remove ourselves in case of malloc failed
}

void ObserverHFI::UpdateRT(const float Ts)
{
    // Here we got the latest Ialphabeta from SenseTask
    const auto foc = GetMotor<FOCMotor>();
    if(foc->IsArmed() && foc->GetControlMode() == MotorControlMode::CTRL_MODE_VELOCITY)
    {
        const auto Iab_real = foc->Ialphabeta_measured;
        // extract high frequency part
        const alphabeta_t Iab_h_now =
        {
            .alpha = (Iab_real.alpha - 2.0f * Iab_last[0].alpha + Iab_last[1].alpha) * 0.25f,
            .beta = (Iab_real.beta - 2.0f * Iab_last[0].beta + Iab_last[1].beta) * 0.25f
        };
        // restore low frequency part, with LPF applied to Ialphabeta_measured
        foc->Ialphabeta_measured.alpha = Ia_lpf.GetOutput((Iab_real.alpha + 2.0f * Iab_last[0].alpha + Iab_last[1].alpha) * 0.25f, Ts);
        foc->Ialphabeta_measured.beta = Ib_lpf.GetOutput((Iab_real.beta + 2.0f * Iab_last[0].beta + Iab_last[1].beta) * 0.25f, Ts);

        // Update historical Iab_real inputs
        Iab_last[1] = Iab_last[0];
        Iab_last[0] = Iab_real;

        // here the inject_sign has been injected in the last cycle, so we should extract envelope first,
        // then flip the sign.
        Iab_envelope.alpha = (Iab_h_now.alpha - Iab_h_last.alpha) * inject_sign;
        Iab_envelope.beta = (Iab_h_now.beta - Iab_h_last.beta) * inject_sign;
        Iab_h_last = Iab_h_now;

        // flip the inject sign.
        inject_sign = wave_injector.GetWaveform(Ts);

        // determine the inject voltage
        Uinject_set = MIN(MIN(foc->GetConfig().max_voltage(), foc->GetBusSense()->voltage) * 0.1f, // 10% vbus
                          foc->GetConfig().calibration_voltage() * 0.8f);
        injector->inject_voltage = Uinject_set * inject_sign;
        pll.GetOutput(Iab_envelope.alpha, Iab_envelope.beta, Ts);

        // apply pll output to encoder
        encoder->single_round_angle_rad = pll.angle_rad;
        const auto delta = encoder->single_round_angle_rad - encoder->last_single_round_angle_rad;
        if(delta > PI) encoder->full_rotations--;
        else if(delta < -PI) encoder->full_rotations++;
        encoder->multi_round_angle_rad = encoder->full_rotations * PI2 + encoder->single_round_angle_rad;
        encoder->last_single_round_angle_rad = encoder->single_round_angle_rad;
        // encoder->angular_speed_rad_s = encoder->speed_lpf.GetOutput(pll.omega_rad_s, Ts);
        encoder->pll_omega_rad_s = pll.omega_rad_s;
    }
    else
    {
        pll.Reset();
        Iab_envelope = {0.0f, 0.0f};
        Iab_last[0] = {0.0f, 0.0f};
        Iab_last[1] = {0.0f, 0.0f};
        Iab_h_last = {0.0f, 0.0f};
        Uinject_set = 0.0f;
        inject_sign = 0.0f; // 0.0f means no injection
        injector->inject_voltage = 0.0f;
        encoder->single_round_angle_rad = 0.0f;
        encoder->angular_speed_rad_s = 0.0f;
        encoder->last_single_round_angle_rad = 0.0f;
        encoder->multi_round_angle_rad = 0.0f;
        encoder->full_rotations = 0;
        encoder->pll_omega_rad_s = 0.0f;
        encoder->speed_lpf.Reset();
    }
}

ObserverHFI::EncoderHFI::EncoderHFI() : EncoderBase("HFIEnc", Encoder::Type::SENSORLESS_ENCODER, 1) {}

void ObserverHFI::EncoderHFI::UpdateMid(const float Ts)
{
    angular_speed_rad_s = speed_lpf.GetOutput(pll_omega_rad_s, Ts);
}

ObserverHFI::InjectorHFI::InjectorHFI() : Task("HFIInj")
{
    RegisterTask(TaskType::RT_TASK);
}

void ObserverHFI::InjectorHFI::UpdateRT(const float Ts)
{
    // inject to Uqd_target
    const auto foc = GetMotor<FOCMotor>();
    foc->Uqd_target.d = inject_voltage;
}
}
