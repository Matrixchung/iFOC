#pragma once

#include "../../Sense/curr_sense_base.hpp"
#include "../../Common/Filter/lowpass_filter.hpp"
#include "main.h"

#if defined(HAL_ADC_MODULE_ENABLED)

// https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/380172/current-sampling-filter-is-essential-in-foc-when-driving-pmsm

namespace iFOC::Sense
{
class TwoShuntCurrSense final : public CurrSenseBase<3>
{
public:
    TwoShuntCurrSense(__IO uint32_t& _JDR_a,
                      __IO uint32_t& _JDR_b,
                      uint16_t& _vref);
    void Update(float Ts) final;
    // void Calibrate() final;
    void UpdateRemainingCurrent(float Ts) final;
    bool IsCalibrated() final;
private:
    __IO uint32_t& JDR_a;
    __IO uint32_t& JDR_b;
    uint16_t& Vrefint;
    float current_sign = 1.0f;
    float zero_a = 0.0f;
    float zero_b = 0.0f;
    iFOC::Filter::LowpassFilter Ia_zero_lpf;
    iFOC::Filter::LowpassFilter Ib_zero_lpf;
    iFOC::Filter::LowpassFilter Ia_lpf;
    iFOC::Filter::LowpassFilter Ib_lpf;
    uint16_t zero_offset_calc_times = 0;
};
}

#endif