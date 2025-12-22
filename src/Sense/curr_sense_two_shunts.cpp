#include "curr_sense_two_shunts.hpp"

#include "../../DataType/board_config.hpp"

#define config iFOC::BoardConfig().GetConfig()

static constexpr uint16_t ZERO_OFFSET_STABLE_TIMES = 1000;

namespace iFOC::Sense
{
CurrSenseTwoShunts::CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a, HAL::ADCPortBase* _JDR_b, bool rev_a, bool rev_b) :
                CurrSenseBase<3>(config.get_current_sense_gain(),
                                config.get_current_sense_shunt_ohm()),
                JDR_a(_JDR_a), JDR_b(_JDR_b),
                Ia_zero_lpf(10), Ib_zero_lpf(10),
                Ia_lpf(config.get_current_sense_f_lp()),
                Ib_lpf(config.get_current_sense_f_lp())
{
    Ia_zero_lpf.output_prev = Ib_zero_lpf.output_prev = JDR_a->GetFullRangeVoltage() * 1000.0f * 0.5f;
    zero_a = Ia_zero_lpf.output_prev;
    zero_b = Ib_zero_lpf.output_prev;
    sign_a = rev_a ? -1.0f : 1.0f;
    sign_b = rev_b ? -1.0f : 1.0f;
}

CurrSenseTwoShunts::CurrSenseTwoShunts(HAL::ADCPortBase* _JDR_a, HAL::ADCPortBase* _JDR_b) : CurrSenseTwoShunts(_JDR_a, _JDR_b, false, false) {}

void CurrSenseTwoShunts::Update(float Ts)
{
    if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES)
    {
        shunt_values[0] = 0.0f;
        shunt_values[1] = 0.0f;
        shunt_values[2] = 0.0f;
        return;
    }
    // Ia + Ib + Ic = 0
    shunt_values[0] = Ia_lpf.GetOutput((JDR_a->GetVoltage_mV() - zero_a) * current_factor_mV * sign_a, Ts);
    shunt_values[1] = Ib_lpf.GetOutput((JDR_b->GetVoltage_mV() - zero_b) * current_factor_mV * sign_b, Ts);
    shunt_values[2] = -shunt_values[0] - shunt_values[1];
}

void CurrSenseTwoShunts::UpdateRemainingCurrent(float Ts)
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

bool CurrSenseTwoShunts::IsCalibrated()
{
    return zero_offset_calc_times >= ZERO_OFFSET_STABLE_TIMES;
}
}
