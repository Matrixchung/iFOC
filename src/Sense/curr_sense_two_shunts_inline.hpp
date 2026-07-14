#pragma once

#include "curr_sense_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"
#include "../Common/Filter/lowpass_filter.hpp"
#include "../DataType/board_config.hpp"

namespace iFOC::Sense
{
template<class X, class Y>
class CurrSenseTwoShuntsInline final : public CurrSenseBase<3>
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(CurrSenseTwoShuntsInline);
public:
    CurrSenseTwoShuntsInline(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b,
                        const bool rev_a,
                        const bool rev_b,
                        const float zero_a_mV,
                        const float zero_b_mV) :
                CurrSenseBase(BoardConfig().GetConfig().get_current_sense_gain(),
                                BoardConfig().GetConfig().get_current_sense_shunt_ohm()),
                JDR_a(_JDR_a), JDR_b(_JDR_b),
                Ia_lpf(BoardConfig().GetConfig().get_current_sense_f_lp()),
                Ib_lpf(BoardConfig().GetConfig().get_current_sense_f_lp())
    {
        // check argument validness
        if constexpr (!(std::is_same_v<X, U> && std::is_same_v<Y, V>) && // Phase U, V scheme
                      !(std::is_same_v<X, U> && std::is_same_v<Y, W>) && // Phase U, W scheme
                      !(std::is_same_v<X, V> && std::is_same_v<Y, W>))   // Phase V, W scheme
        {
            static_assert(false, "CurrSenseTwoShunts phase argument invalid!");
        }
        coeff_a = rev_a ? -current_factor_mV : current_factor_mV;
        coeff_b = rev_b ? -current_factor_mV : current_factor_mV;
        if(zero_a_mV > 1.0f && zero_b_mV > 1.0f)
        {
            zero_a = zero_a_mV;
            zero_b = zero_b_mV;
        }
        else
        {
            zero_a = zero_b = JDR_a->GetFullRangeVoltage() * 500.0f;
        }
    }

    CurrSenseTwoShuntsInline(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b) : CurrSenseTwoShuntsInline(_JDR_a, _JDR_b, false, false, 0.0f, 0.0f) {}

    CurrSenseTwoShuntsInline(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b, const bool rev_a, const bool rev_b) : CurrSenseTwoShuntsInline(_JDR_a, _JDR_b, rev_a, rev_b, 0.0f, 0.0f) {}

    void Update(const float Ts) override
    {
        // Ia + Ib + Ic = 0, U/V/W
        if constexpr (std::is_same_v<X, U> && std::is_same_v<Y, V>) // Phase U(a), V(b)
        {
            shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * coeff_a, Ts);
            shunt_values[1] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * coeff_b, Ts);
            shunt_values[2] = -shunt_values[0] - shunt_values[1];
        }
        else if constexpr (std::is_same_v<X, U> && std::is_same_v<Y, W>) // Phase U(a), W(b)
        {
            shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * coeff_a, Ts);
            shunt_values[2] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * coeff_b, Ts);
            shunt_values[1] = -shunt_values[0] - shunt_values[2];
        }
        else if constexpr (std::is_same_v<X, V> && std::is_same_v<Y, W>) // Phase V(a), W(b)
        {
            shunt_values[1] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * coeff_a, Ts);
            shunt_values[2] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * coeff_b, Ts);
            shunt_values[0] = -shunt_values[1] - shunt_values[2];
        }
    }

    bool IsCalibrated() override
    {
        return true;
    }
private:
    HAL::ADCPortBase* JDR_a;
    HAL::ADCPortBase* JDR_b;
    Filter::LowpassFilter Ia_lpf;
    Filter::LowpassFilter Ib_lpf;
    float zero_a = 0.0f;
    float zero_b = 0.0f;
    float coeff_a = 1.0f;
    float coeff_b = 1.0f;
};
}