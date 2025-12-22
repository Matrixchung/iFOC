#pragma once

#include "curr_sense_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"
#include "../Common/Filter/lowpass_filter.hpp"

// https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/380172/current-sampling-filter-is-essential-in-foc-when-driving-pmsm

namespace iFOC::Sense
{
class CurrSenseTwoShunts final : public CurrSenseBase<3>
{
    DELETE_COPY_CONSTRUCTOR(CurrSenseTwoShunts);
public:
    CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b,
                        bool rev_a,
                        bool rev_b);
    CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b);
    void Update(float Ts) override;
    void UpdateRemainingCurrent(float Ts) override;
    bool IsCalibrated() override;
private:
    uint16_t zero_offset_calc_times = 0;
    HAL::ADCPortBase* JDR_a;
    HAL::ADCPortBase* JDR_b;
    real_t zero_a = 0.0f;
    real_t zero_b = 0.0f;
    real_t sign_a = 1.0f;
    real_t sign_b = 1.0f;
    Filter::LowpassFilter Ia_zero_lpf;
    Filter::LowpassFilter Ib_zero_lpf;
    Filter::LowpassFilter Ia_lpf;
    Filter::LowpassFilter Ib_lpf;
};
}