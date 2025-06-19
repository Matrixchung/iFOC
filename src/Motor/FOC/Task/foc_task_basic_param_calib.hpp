#pragma once

#include "../foc_motor.hpp"
#include "foc_math.hpp"
#include "trajectory_controller.hpp"
#include "wave_injector.hpp"
#include "foc_harmonic_regulator.hpp"
#include "sliding_filter.hpp"

namespace iFOC::FOC
{
/// Goal: calculate Rs, Ls, flux to config.
class BasicParamCalibTask final : public Task
{
// private:
public:
    using Bridge = Driver::FOCDriverBase::Bridge;
    struct Rs_est_t
    {
        uint32_t TARGET_LOOP_COUNT = 40000;
        static constexpr float Ki = 4.0f;
        float target_current = 0.0f;
        float voltage_diff[3] = {0.0f};
        float Rs_result[3] = {0.0f};
        uint32_t loop_count = 0;
    };
    struct Ls_est_t
    {
        uint32_t TARGET_LOOP_COUNT = 20000;
        float test_voltage = 0.0f;
        float currents[3][2] = {{0.0f}};
        float Ls_result[3] = {0.0f};
        uint8_t pwm_freq_div_1k = 1;
        uint32_t loop_count = 0;
    };
    /// Reference: Y. Wang et al., "A Robust DPCC for IPMSM Based on a Full Parameter Identification Method,"
    ///            in IEEE Transactions on Industrial Electronics, vol. 70, no. 8, pp. 7695-7705, Aug. 2023
    /// Reference: https://zhuanlan.zhihu.com/p/685863710
    struct flux_est_t
    {
        TrajController traj;
        HarmonicRegulator harmonic_reg;
        Filter::SlidingFilter iqf_filter;
        Filter::SlidingFilter idf_filter;
        Filter::SlidingFilter uqf_star_filter;
        Filter::SlidingFilter udf_star_filter;
        Filter::LowpassFilter flux_result_filter;
        float cruise_we = 0.0f;
        float w0_9 = 0.0f;
        // float phase_comp_9 = 0.0f;
        // float wc = 0.0f;
        // float w0_9_Ts = 0.0f;
        float sin_w0_9_Ts = 0.0f;
        float cos_w0_9_Ts = 0.0f;
        float sin_comp_9 = 0.0f;
        float cos_comp_9 = 0.0f;
        float harmonic_reg_9_output = 0.0f;
        float iqf = 0.0f;
        float idf = 0.0f;
        float uqf_star = 0.0f;
        float udf_star = 0.0f;
    };
    union data_t
    {
        Rs_est_t Rs_est;
        Ls_est_t Ls_est;
        flux_est_t flux_est;
    };
    enum class EstStage : uint8_t
    {
        NONE = 0,
        PHASE_RESISTANCE_START,
        PHASE_RESISTANCE_TESTING_U,
        PHASE_RESISTANCE_TESTED_U,
        PHASE_RESISTANCE_TESTING_V,
        PHASE_RESISTANCE_TESTED_V,
        PHASE_RESISTANCE_TESTING_W,
        PHASE_RESISTANCE_TESTED_W,

        PHASE_INDUCTANCE_START,
        PHASE_INDUCTANCE_TESTING_U,
        PHASE_INDUCTANCE_TESTED_U,
        PHASE_INDUCTANCE_TESTING_V,
        PHASE_INDUCTANCE_TESTED_V,
        PHASE_INDUCTANCE_TESTING_W,
        PHASE_INDUCTANCE_TESTED_W,

        FLUX_LINKAGE_START,
        FLUX_LINKAGE_TESTING,
        FLUX_LINKAGE_TESTED,
    };
    EstStage stage = EstStage::NONE;
    data_t data{};
    WaveInjector wave{WaveInjector::WaveType::SINUSOIDAL};
public:
    BasicParamCalibTask();
    void UpdateRT(float Ts) final;
    void InitNormal() final;
    void UpdateNormal() final;
};
}