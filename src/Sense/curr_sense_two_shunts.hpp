#pragma once

#include "curr_sense_base.hpp"
#include "../Common/Interface/adc_port_base.hpp"
#include "../Common/Filter/lowpass_filter.hpp"
#include "../../DataType/board_config.hpp"

// https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/380172/current-sampling-filter-is-essential-in-foc-when-driving-pmsm

namespace iFOC::Sense
{
template<class X, class Y>
class CurrSenseTwoShunts final : public CurrSenseBase<3>
{
    DELETE_COPY_CONSTRUCTOR(CurrSenseTwoShunts);
private:
    static constexpr uint16_t ZERO_OFFSET_STABLE_TIMES = 1000;
public:
    CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b,
                        bool rev_a,
                        bool rev_b) :
                CurrSenseBase(BoardConfig().GetConfig().get_current_sense_gain(),
                                BoardConfig().GetConfig().get_current_sense_shunt_ohm()),
                JDR_a(_JDR_a), JDR_b(_JDR_b),
                Ia_zero_lpf(10), Ib_zero_lpf(10),
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
        Ia_zero_lpf.output_prev = Ib_zero_lpf.output_prev = JDR_a->GetFullRangeVoltage() * 1000.0f * 0.5f;
        zero_a = Ia_zero_lpf.output_prev;
        zero_b = Ib_zero_lpf.output_prev;
        sign_a = rev_a ? -1.0f : 1.0f;
        sign_b = rev_b ? -1.0f : 1.0f;
    }

    CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a,
                        HAL::ADCPortBase* _JDR_b) : CurrSenseTwoShunts(_JDR_a, _JDR_b, false, false) {}


    void Update(float Ts) override
    {
        if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES)
        {
            shunt_values[0] = 0.0f;
            shunt_values[1] = 0.0f;
            shunt_values[2] = 0.0f;
            return;
        }
        // Ia + Ib + Ic = 0, U/V/W
        if constexpr (std::is_same_v<X, U> && std::is_same_v<Y, V>) // Phase U(a), V(b)
        {
            shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * current_factor_mV * sign_a, Ts);
            shunt_values[1] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * current_factor_mV * sign_b, Ts);
            shunt_values[2] = -shunt_values[0] - shunt_values[1];
        }
        else if constexpr (std::is_same_v<X, U> && std::is_same_v<Y, W>) // Phase U(a), W(b)
        {
            shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * current_factor_mV * sign_a, Ts);
            shunt_values[2] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * current_factor_mV * sign_b, Ts);
            shunt_values[1] = -shunt_values[0] - shunt_values[2];
        }
        else if constexpr (std::is_same_v<X, V> && std::is_same_v<Y, W>) // Phase V(a), W(b)
        {
            shunt_values[1] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * current_factor_mV * sign_a, Ts);
            shunt_values[2] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * current_factor_mV * sign_b, Ts);
            shunt_values[0] = -shunt_values[1] - shunt_values[2];
        }
    }

    void UpdateRemainingCurrent(float Ts) override
    {
        const real_t Va = JDR_a->GetVoltage_mV();
        const real_t Vb = JDR_b->GetVoltage_mV();
        const real_t Vhalf = JDR_a->GetFullRangeVoltage() * 1000.0f * 0.5f;
        const real_t Vlow = Vhalf * 0.5f;
        const real_t Vhigh = Vhalf * 1.5f;
        if(BETWEEN(Va, Vlow, Vhigh) && BETWEEN(Vb, Vlow, Vhigh))
        {
            if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES) zero_offset_calc_times++;
            zero_a = Ia_zero_lpf.GetOutput(Va, Ts);
            zero_b = Ib_zero_lpf.GetOutput(Vb, Ts);
        }
    }

    bool IsCalibrated() override
    {
        return zero_offset_calc_times >= ZERO_OFFSET_STABLE_TIMES;
    }
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