#include "at32wk_foc_driver_6pwm.hpp"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED)

#include "../../DataType/board_config.hpp"

namespace iFOC::Driver
{
#define config iFOC::BoardConfig().GetConfig()

FOCDriver6PWM::FOCDriver6PWM(tmr_type* _htim) : htim(_htim) {};

FuncRetCode FOCDriver6PWM::Init(bool initCNT)
{
    tmr_reset(htim);

    DisableAllOutputs();

    tmr_counter_enable(htim, FALSE);

    auto pwm_freq = config.pwm_wave_freq();
    if(pwm_freq == 0) pwm_freq = 20000; // fallback to 20KHz
    const auto timer_base_clock = HAL::GetCoreClockHz();
    uint32_t arr = (timer_base_clock / pwm_freq) / 2; // center aligned
    max_compare = arr;

    tmr_cnt_dir_set(htim, TMR_COUNT_TWO_WAY_1);
    tmr_repetition_counter_set(htim, 0);
    tmr_clock_source_div_set(htim, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(htim, TRUE);
    tmr_base_init(htim, arr, 0); // arr set, divider = 0

    tmr_sub_sync_mode_set(htim, FALSE);

    tmr_primary_mode_select(htim, TMR_PRIMARY_SEL_OVERFLOW); // Set TRGO

    tmr_output_config_type tmr_output_struct
    {
        .oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B,
        .oc_idle_state = FALSE, // CHx OCIDLESTATE_LOW
        .occ_idle_state = TRUE, // CHxN OCIDLESTATE HIGH
        .oc_polarity = TMR_OUTPUT_ACTIVE_HIGH,
        .occ_polarity = TMR_OUTPUT_ACTIVE_HIGH,
        .oc_output_state = TRUE,
        .occ_output_state = TRUE
    };

    tmr_output_channel_config(htim, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    htim->c1dt = 0;
    tmr_output_channel_buffer_enable(htim, TMR_SELECT_CHANNEL_1, TRUE);

    tmr_output_channel_config(htim, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    htim->c2dt = 0;
    tmr_output_channel_buffer_enable(htim, TMR_SELECT_CHANNEL_2, TRUE);

    tmr_output_channel_config(htim, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
    htim->c3dt = 0;
    tmr_output_channel_buffer_enable(htim, TMR_SELECT_CHANNEL_3, TRUE);

    // configure Break & Dead Time
    tmr_brkdt_config_type tmr_brkdt_struct
    {
        .deadtime = 10,
        .brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW,
        .wp_level = TMR_WP_OFF,
        .auto_output_enable = FALSE,
        .fcsoen_state = TRUE,
        .fcsodis_state = TRUE,
        .brk_enable = FALSE
    };
    tmr_brkdt_config(htim, &tmr_brkdt_struct);

    EnableBridges(Bridge::HB_U, Bridge::HB_V, Bridge::HB_W,
                  Bridge::LB_U, Bridge::LB_V, Bridge::LB_W);

    SetOutput3CHRaw(0, 0, 0);

    if(initCNT)
    {
        EnableAllOutputs();
        tmr_counter_enable(htim, TRUE);
    }
    return FuncRetCode::OK;
}

void FOCDriver6PWM::EnableBridge(Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: htim->cctrl_bit.c1en = true; break;
        case Bridge::HB_V: htim->cctrl_bit.c2en = true; break;
        case Bridge::HB_W: htim->cctrl_bit.c3en = true; break;
        case Bridge::LB_U: htim->cctrl_bit.c1cen = true; break;
        case Bridge::LB_V: htim->cctrl_bit.c2cen = true; break;
        case Bridge::LB_W: htim->cctrl_bit.c3cen = true; break;
    }
}

void FOCDriver6PWM::DisableBridge(Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: htim->cctrl_bit.c1en = false; break;
        case Bridge::HB_V: htim->cctrl_bit.c2en = false; break;
        case Bridge::HB_W: htim->cctrl_bit.c3en = false; break;
        case Bridge::LB_U: htim->cctrl_bit.c1cen = false; break;
        case Bridge::LB_V: htim->cctrl_bit.c2cen = false; break;
        case Bridge::LB_W: htim->cctrl_bit.c3cen = false; break;
    }
}

real_t FOCDriver6PWM::GetDeadTime()
{
    real_t dead_time = 0.0f;
    const auto timer_base_clock = HAL::GetCoreClockHz();
    float Tdts = 1.0f / (float)timer_base_clock;
    auto DTC = htim->brk_bit.dtc;
    if(DTC & (1 << 7))
    {
        if(DTC & (1 << 6))
        {
            if(DTC & (1 << 5))
            {
                dead_time = (32.0f + (float)(DTC & 0x1F)) * 16.0f * Tdts;
            }
            else dead_time = (32.0f + (float)(DTC & 0x1F)) * 8.0f * Tdts;
        }
        else dead_time = (64.0f + (float)(DTC & 0x3F)) * 2.0f * Tdts;
    }
    else dead_time = (float)DTC * Tdts;
    return dead_time;
}
}

#endif