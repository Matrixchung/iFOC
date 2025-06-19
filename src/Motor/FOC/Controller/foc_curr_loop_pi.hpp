#pragma once

#include "foc_curr_loop_base.hpp"
#include "foc_harmonic_regulator.hpp"
#include "pi_controller.hpp"

namespace iFOC::FOC
{
class CurrLoopPI final : public CurrLoopBase
{
public:
    CurrLoopPI();
    HarmonicRegulator q_harmonic_reg_6;
    HarmonicRegulator q_harmonic_reg_12;
    HarmonicRegulator d_harmonic_reg_6;
    HarmonicRegulator d_harmonic_reg_12;
    PIController q_pi;
    PIController d_pi;
    bool enable_harmonic_regulator = true;
    bool enable_pi_feedforward = true;
    bool enable_flux_feedforward = true;
    void InitCurrLoop() final;
    void ResetCurrLoop() final;
    void UpdateCurrLoop(float Ts) final;
};
}