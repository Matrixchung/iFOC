#include "curr_sense_three_shunts.hpp"
#include "../DataType/board_config.hpp"

#define config iFOC::BoardConfig().GetConfig()

constexpr uint16_t ZERO_OFFSET_STABLE_TIMES = 1000;

namespace iFOC::Sense
{
CurrSenseThreeShunts::CurrSenseThreeShunts(HAL::ADCPortBase* _a, HAL::ADCPortBase* _b, HAL::ADCPortBase* _c,
    const bool rev_a, const bool rev_b, const bool rev_c) :
        CurrSenseBase(config.get_current_sense_gain(), config.get_current_sense_shunt_ohm()),
        JDR_a(_a), JDR_b(_b), JDR_c(_c),
        Ia_zero_lpf(10), Ib_zero_lpf(10), Ic_zero_lpf(10),
        Ia_lpf(config.get_current_sense_f_lp()),
        Ib_lpf(config.get_current_sense_f_lp()),
        Ic_lpf(config.get_current_sense_f_lp())
{
    Ia_zero_lpf.output_prev = Ib_zero_lpf.output_prev = Ic_zero_lpf.output_prev = JDR_a->GetFullRangeVoltage() * 1000.0f * 0.5f;
    zero_a = zero_b = zero_c = Ia_zero_lpf.output_prev;
    coeff_a = rev_a ? -current_factor_mV : current_factor_mV;
    coeff_b = rev_b ? -current_factor_mV : current_factor_mV;
    coeff_c = rev_c ? -current_factor_mV : current_factor_mV;
}

void CurrSenseThreeShunts::Update(const float Ts)
{
    if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES)
    {
        shunt_values[0] = 0.0f;
        shunt_values[1] = 0.0f;
        shunt_values[2] = 0.0f;
        return;
    }
    shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * coeff_a, Ts);
    shunt_values[1] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * coeff_b, Ts);
    shunt_values[2] = Ic_lpf.GetOutput((JDR_c->GetVoltage_mV() - zero_c) * coeff_c, Ts);
}

void CurrSenseThreeShunts::UpdateRemainingCurrent(const float Ts)
{
    const real_t Va = JDR_a->GetVoltage_mV();
    const real_t Vb = JDR_b->GetVoltage_mV();
    const real_t Vc = JDR_c->GetVoltage_mV();
    const real_t Vhalf = JDR_a->GetFullRangeVoltage() * 1000.0f * 0.5f;
    const real_t Vlow = Vhalf * 0.5f;
    const real_t Vhigh = Vlow + Vhalf;
    if(BETWEEN(Va, Vlow, Vhigh) && BETWEEN(Vb, Vlow, Vhigh) && BETWEEN(Vc, Vlow, Vhigh))
    {
        if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES) zero_offset_calc_times++;
        zero_a = Ia_zero_lpf.GetOutput(Va, Ts);
        zero_b = Ib_zero_lpf.GetOutput(Vb, Ts);
        zero_c = Ic_zero_lpf.GetOutput(Vc, Ts);
    }
}

bool CurrSenseThreeShunts::IsCalibrated()
{
    return zero_offset_calc_times >= ZERO_OFFSET_STABLE_TIMES;
}
}
