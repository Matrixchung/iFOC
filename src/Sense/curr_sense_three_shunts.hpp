#pragma once

#include "curr_sense_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"
#include "../Common/Filter/lowpass_filter.hpp"

namespace iFOC::Sense
{
class CurrSenseThreeShunts final : public CurrSenseBase<3>
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(CurrSenseThreeShunts);
public:
    CurrSenseThreeShunts() = delete;
    CurrSenseThreeShunts(HAL::ADCPortBase* _a, HAL::ADCPortBase* _b, HAL::ADCPortBase* _c,
                         bool rev_a, bool rev_b, bool rev_c);
    void Update(float Ts) override;
    void UpdateRemainingCurrent(float Ts) override;
    bool IsCalibrated() override;
private:
    HAL::ADCPortBase* JDR_a;
    HAL::ADCPortBase* JDR_b;
    HAL::ADCPortBase* JDR_c;
    real_t zero_a = 0.0f;
    real_t zero_b = 0.0f;
    real_t zero_c = 0.0f;
    Filter::LowpassFilter Ia_zero_lpf;
    Filter::LowpassFilter Ib_zero_lpf;
    Filter::LowpassFilter Ic_zero_lpf;
    Filter::LowpassFilter Ia_lpf;
    Filter::LowpassFilter Ib_lpf;
    Filter::LowpassFilter Ic_lpf;
    uint16_t zero_offset_calc_times = 0;
    // int8_t sign_a = 1;
    // int8_t sign_b = 1;
    // int8_t sign_c = 1;
    real_t coeff_a = 1.0f;
    real_t coeff_b = 1.0f;
    real_t coeff_c = 1.0f;
};
}