#pragma once

#include "../foc_motor.hpp"
#include "../../../Encoder/encoder_base.hpp"
#include "../../../Common/Filter/lowpass_filter.hpp"
#include "../../../Common/Filter/qpll.hpp"
#include "../../../Common/wave_injector.hpp"

/*
 * Y. -D. Yoon, S. -K. Sul, S. Morimoto and K. Ide, "High-Bandwidth Sensorless Algorithm for AC Machines
 * Based on Square-Wave-Type Voltage Injection," in IEEE Transactions on Industry Applications, vol. 47,
 * no. 3, pp. 1361-1370, May-June 2011, doi: 10.1109/TIA.2011.2126552.
 */

/*
 * ObserverHFI consists of three major parts:
 * 1)    Main observer task: extract high frequency signal from stator frame Ialpha/beta,
 *                           passing base frequency signal (and/or lowpass filtered) Ialpha/beta to Park transform,
 *                           doing PLLs, determining polarity, updating estimated electric angle
 *                           AFTER got latest Ialphabeta, BEFORE transforming Ialphabeta to Iqd with given elec angle (Park Trans.)
 *                           SenseTask -> HFIMain -> ... -> EncArbiter -> Park -> ...
 *
 * 2) HFI-simulated encoder: acting as an encoder instance which provides PLL-calculated elec_angle_rad/elec_omega_rad_s
 *                           the decision on HOW and WHEN to use the angle provided by HFI, is made by EncArbiter.
 *                           other encoders... -> HFIEnc -> other encoders... -> EncArbiter -> ...
 *
 * 3)          Uqd injector: modifying final Uqd_target result, to inject +- square voltage to Ud.
 *                           AFTER calculated Uqd_target by Current Loop, BEFORE applying Uqd_target to WaveGenerator.
 *                           ... -> CurrLoopPI -> HFIInj -> WaveGen
 *
 *           Final sequence: ... -> HFIMain -> ... -> HFIEnc -> ... -> HFIInj -> WaveGen
 *                                   \|/     angle     /|\               /|\
 *                                    |_________________|     Uinject     |
 *                                    |___________________________________|
 *
*                            TASK LINE: SenseTask -> (HFIMain) -> Encoders... -> (HFIEnc) -> EncArbiter -> Park -> ... -> (HFIInj)
 *                           To eliminate the angle delay
 */

namespace iFOC::FOC
{
class ObserverHFI final : public Task
{
    OVERRIDE_NEW();
public:
    ObserverHFI();
    ~ObserverHFI() override;
    void InitRT() override;
    void UpdateRT(float Ts) override;
// private:
    class EncoderHFI final : public Encoder::EncoderBase
    {
        OVERRIDE_NEW();
    public:
        friend class ObserverHFI;
        EncoderHFI();
        void UpdateMid(float Ts) override;
        float last_single_round_angle_rad = 0.0f;
        float pll_omega_rad_s = 0.0f;
    };
    class InjectorHFI final : public Task
    {
        OVERRIDE_NEW();
    public:
        friend class ObserverHFI;
        InjectorHFI();
        void UpdateRT(float Ts) override;
        float inject_voltage = 0.0f;
    };
    WaveInjector wave_injector{};
    Filter::LowpassFilter Ia_lpf;
    Filter::LowpassFilter Ib_lpf;
    Filter::QPLL pll{};
    EncoderHFI* encoder = nullptr;
    InjectorHFI* injector = nullptr;
    alphabeta_t Iab_envelope{};
    alphabeta_t Iab_last[2]{};
    alphabeta_t Iab_h_last{};
    float Uinject_set = 0.0f;
    float inject_sign = 0.0f;
};
}