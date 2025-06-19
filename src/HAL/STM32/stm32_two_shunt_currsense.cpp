#include "stm32_two_shunt_currsense.hpp"

#if defined(HAL_ADC_MODULE_ENABLED)

#include "../../DataType/board_config.hpp"

namespace iFOC::Sense
{
#define config iFOC::BoardConfig.GetConfig()

static constexpr uint16_t ZERO_OFFSET_STABLE_TIMES = 1000;

static constexpr bool IS_VALID_ZERO(const auto val)
{
#if defined(LL_ADC_RESOLUTION_16B)
    return BETWEEN(val, 15000, 45000);
#elif defined(LL_ADC_RESOLUTION_12B)
    return BETWEEN(val, 1500, 2500);
#else
#error "Platform not supported!"
#endif
}

TwoShuntCurrSense::TwoShuntCurrSense(volatile uint32_t &_JDR_a,
                                     volatile uint32_t &_JDR_b,
                                     uint16_t &_vref) :
        CurrSenseBase<3>(config.get_current_sense_gain(),
                         config.get_current_sense_shunt_ohm()),
        JDR_a(_JDR_a), JDR_b(_JDR_b), Vrefint(_vref),
        Ia_zero_lpf(10), Ib_zero_lpf(10),
        Ia_lpf( config.get_current_sense_f_lp()),
        Ib_lpf(config.get_current_sense_f_lp())
{
    // if(config.get_current_sense_is_calibrated())
    // {
    //     if(!_const::IS_VALID_ZERO(config.get_current_sense_zero_a()) ||
    //        !_const::IS_VALID_ZERO(config.get_current_sense_zero_b()) )
    //     {
    //         config.set_current_sense_is_calibrated(false);
    //         // iFOC::misconfigured = true;
    //     }
    //     else
    //     {
    //         zero_a = config.get_current_sense_zero_a();
    //         zero_b = config.get_current_sense_zero_b();
    //     }
    //     // else UpdateZeroOffset();
    // }
#if defined(LL_ADC_RESOLUTION_16B)
    Ia_zero_lpf.output_prev = Ib_zero_lpf.output_prev = 32767.0f;
#elif defined(LL_ADC_RESOLUTION_12B)
    Ia_zero_lpf.output_prev = Ib_zero_lpf.output_prev = 2047.0f;
#else
#error "Platform not supported!"
#endif
    current_sign = config.current_sense_dir_reversed() ? -1.0f : 1.0f;
}

void TwoShuntCurrSense::Update(float Ts)
{
    // keep current stable for initial condition
    if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES)
    {
        shunt_values.fill(0.0f);
        return;
    }
#if defined(LL_ADC_RESOLUTION_16B)
    auto vref_mpy_factor_and_sign = current_sign * current_factor * (float)__LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_16B) / 65535.0f;
#elif defined(LL_ADC_RESOLUTION_12B)
    auto vref_mpy_factor_and_sign = current_sign * current_factor * (float)(__LL_ADC_CALC_VREFANALOG_VOLTAGE(Vrefint, LL_ADC_RESOLUTION_12B)) / 4095.0f;
#endif
    // Ia + Ib + Ic = 0;
    shunt_values[0] = Ia_lpf.GetOutput((float)((float)JDR_a - zero_a) * vref_mpy_factor_and_sign, Ts);
    shunt_values[1] = Ib_lpf.GetOutput((float)((float)JDR_b - zero_b) * vref_mpy_factor_and_sign, Ts);
    shunt_values[2] = -shunt_values[0] - shunt_values[1];
}

// void TwoShuntCurrSense::Calibrate()
// {
//     uint32_t a_zero_total = 0, b_zero_total = 0;
//     uint16_t calibration_times = _const::CURR_SENSE_ZERO_CALIBRATION_TIMES;
//     while(calibration_times--)
//     {
//         a_zero_total += (uint32_t)JDR_a;
//         b_zero_total += (uint32_t)JDR_b;
//         vTaskDelay(1);
//     }
//     a_zero_total /= _const::CURR_SENSE_ZERO_CALIBRATION_TIMES;
//     b_zero_total /= _const::CURR_SENSE_ZERO_CALIBRATION_TIMES;
//     if(!_const::IS_VALID_ZERO(a_zero_total) || !_const::IS_VALID_ZERO(b_zero_total))
//     {
//         config.set_current_sense_zero_a(0);
//         config.set_current_sense_zero_b(0);
//         config.set_current_sense_is_calibrated(false);
//         misconfigured_area |= to_underlying(MisconfiguredArea::CURR_SENSE_CALIB_VALUE_OUT_OF_RANGE);
//         return;
//     }
//     config.set_current_sense_zero_a(a_zero_total);
//     config.set_current_sense_zero_b(b_zero_total);
//     config.set_current_sense_is_calibrated(true);
//     UpdateZeroOffset();
// }

void TwoShuntCurrSense::UpdateRemainingCurrent(float Ts)
{
//     if(config.get_current_sense_is_calibrated())
//     {
//         zero_a = config.get_current_sense_zero_a();
//         zero_b = config.get_current_sense_zero_b();
//     }
//     else
//     {
// #if defined(LL_ADC_RESOLUTION_16B)
//         zero_a = zero_b = 32767;
// #elif defined(LL_ADC_RESOLUTION_12B)
//         zero_a = zero_b = 2047;
// #endif
//     }
    if(IS_VALID_ZERO(JDR_a) && IS_VALID_ZERO(JDR_b))
    {
        if(zero_offset_calc_times < ZERO_OFFSET_STABLE_TIMES) zero_offset_calc_times++;
        zero_a = Ia_zero_lpf.GetOutput((float)JDR_a, Ts);
        zero_b = Ib_zero_lpf.GetOutput((float)JDR_b, Ts);
    }
}

bool TwoShuntCurrSense::IsCalibrated()
{
    return zero_offset_calc_times >= ZERO_OFFSET_STABLE_TIMES;
}

}

#endif