#include "at32wk_dc_driver_1pwm.hpp"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED) && defined(GPIO_MODULE_ENABLED)

#include "../../DataType/board_config.hpp"

namespace iFOC::Driver
{
#define config iFOC::BoardConfig().GetConfig()

DCDriver1PWM::DCDriver1PWM(tmr_type* _htim,  tmr_channel_select_type _ch) : DCDriver1PWM(_htim, _ch, nullptr) {}

DCDriver1PWM::DCDriver1PWM(tmr_type* _htim, tmr_channel_select_type _ch, HAL::GPIOBase* _dir) : htim(_htim), channel(_ch), dir_gpio(_dir) {}

FuncRetCode DCDriver1PWM::Init(bool initTIM)
{
    tmr_reset(htim);

    DisableAllOutputs();

    tmr_output_enable(htim, FALSE);
    tmr_counter_enable(htim, FALSE);

    auto pwm_freq = config.pwm_wave_freq();
    const auto timer_base_clock = HAL::GetCoreClockHz();
    uint32_t arr = (timer_base_clock / pwm_freq) - 1;
    max_compare = arr;

    tmr_cnt_dir_set(htim, TMR_COUNT_UP);
    tmr_clock_source_div_set(htim, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(htim, TRUE);
    tmr_base_init(htim, arr, 0);

    tmr_sub_sync_mode_set(htim, FALSE);
    tmr_primary_mode_select(htim, TMR_PRIMARY_SEL_RESET);

    tmr_output_config_type tmr_output_struct
    {
        .oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A,
        .oc_idle_state = FALSE, // CHx OCIDLESTATE_LOW
        .occ_idle_state = TRUE, // CHxN OCIDLESTATE HIGH
        .oc_polarity = TMR_OUTPUT_ACTIVE_HIGH,
        .occ_polarity = TMR_OUTPUT_ACTIVE_HIGH,
        .oc_output_state = TRUE,
        .occ_output_state = TRUE
    };

    tmr_output_channel_config(htim, channel, &tmr_output_struct);
    tmr_channel_value_set(htim, channel, 0);

    if(dir_gpio)
    {
        dir_gpio->ModeOutPP();
        dir_gpio->Clear();
    }

    if(initTIM)
    {
        EnableAllOutputs();
        tmr_output_enable(htim, TRUE);
        tmr_counter_enable(htim, TRUE);
    }
    return FuncRetCode::OK;
}

void DCDriver1PWM::SetOutputRaw(uint32_t ch, uint8_t dir)
{
    tmr_channel_value_set(htim, channel, ch);
    if(dir_gpio)
    {
        if(dir) dir_gpio->Set();
        else dir_gpio->Clear();
    }
}
}

#endif