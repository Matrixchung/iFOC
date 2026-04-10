#include "gd32_foc_driver_6pwm.hpp"

#if defined(GD32_ENV)

#include "../../DataType/board_config.hpp"

#if defined(GD32G5X3) && defined(TIMER_CTL0_ADMEN)
#define TIM_DITHERING_MODE_EN // ARR * 16
#endif

// TIM Break: https://blog.csdn.net/hwytree/article/details/125603354
// https://shequ.stmicroelectronics.cn/thread-635826-1-1.html
// https://www.st.com/resource/en/application_note/dm00080497.pdf

namespace iFOC::Driver
{
#define config iFOC::BoardConfig().GetConfig()

FOCDriver6PWM::FOCDriver6PWM(const uint32_t _htim) : htim(_htim) {}

FuncRetCode FOCDriver6PWM::Init(bool initCNT)
{
    timer_deinit(htim);
    DisableAllOutputs();

    timer_parameter_struct tim_init_struct;
    timer_oc_parameter_struct tim_oc_param_struct;
    timer_free_complementary_parameter_struct tim_free_comp_param_struct;
    timer_break_parameter_struct tim_break_param_struct;

    timer_struct_para_init(&tim_init_struct);
    timer_channel_output_struct_para_init(&tim_oc_param_struct);
    timer_free_complementary_struct_para_init(&tim_free_comp_param_struct);
    timer_break_struct_para_init(&tim_break_param_struct);

    auto pwm_freq = config.pwm_wave_freq();
    if(pwm_freq == 0) pwm_freq = 20000; // fallback to 20KHz
    const auto timer_base_clock = HAL::GetCoreClockHz();
#ifdef TIM_DITHERING_MODE_EN
    max_compare = ((timer_base_clock * 16 / pwm_freq) / 2);
    timer_adjustment_mode_config(htim, ENABLE);
#else
    const uint32_t arr = ((timer_base_clock / pwm_freq) / 2);
    max_compare = arr;
#endif

    timer_auto_reload_shadow_enable(htim);
    timer_counter_initial_register_config(htim, DISABLE);
    timer_upif_backup_config(htim, DISABLE);
    timer_master_slave_mode_config(htim, TIMER_MASTER_SLAVE_MODE_DISABLE);

    timer_master_output0_trigger_source_select(htim, TIMER_TRI_OUT0_SRC_UPDATE); // Set TRGO to OVERFLOW(UPDATE) event

    tim_init_struct.prescaler = 0;
    tim_init_struct.alignedmode = TIMER_COUNTER_CENTER_DOWN;
    tim_init_struct.counterdirection = TIMER_COUNTER_UP;
#ifdef TIM_DITHERING_MODE_EN
    tim_init_struct.period = max_compare >> 4U;
#else
    tim_init_struct.period = max_compare;
#endif
    tim_init_struct.repetitioncounter = 0;
    tim_init_struct.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(htim, &tim_init_struct);

#ifdef TIM_DITHERING_MODE_EN
    timer_auto_reload_fract_value_config(htim, (max_compare & 0xFU));
#endif

    tim_oc_param_struct.outputstate = TIMER_CCX_ENABLE;
    tim_oc_param_struct.outputnstate = TIMER_CCXN_ENABLE;
    tim_oc_param_struct.ocpolarity = TIMER_OC_POLARITY_HIGH;
    tim_oc_param_struct.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    tim_oc_param_struct.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    tim_oc_param_struct.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

    tim_free_comp_param_struct.freecomstate = TIMER_FCCHP_STATE_DISABLE;

    timer_channel_output_config(htim, TIMER_CH_0, &tim_oc_param_struct);
    timer_channel_output_mode_config(htim, TIMER_CH_0, TIMER_OC_MODE_PWM1); // PWM Mode B
    timer_channel_output_pulse_value_config(htim, TIMER_CH_0, 0);
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_0, 0);
    timer_channel_output_shadow_config(htim, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_clear_config(htim, TIMER_CH_0, TIMER_OC_CLEAR_DISABLE);
    timer_channel_output_compare_fast_config(htim, TIMER_CH_0, TIMER_OC_FAST_DISABLE);
    timer_channel_free_complementary_config(htim, TIMER_CH_0, &tim_free_comp_param_struct);
    timer_channel_dead_time_config(htim, TIMER_CH_0, ENABLE);
    timer_channel_composite_pwm_mode_config(htim, TIMER_CH_0, DISABLE);
    timer_output_match_pulse_select(htim, TIMER_CH_0, TIMER_PULSE_OUTPUT_NORMAL);

    timer_channel_output_config(htim, TIMER_CH_1, &tim_oc_param_struct);
    timer_channel_output_mode_config(htim, TIMER_CH_1, TIMER_OC_MODE_PWM1); // PWM Mode B
    timer_channel_output_pulse_value_config(htim, TIMER_CH_1, 0);
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_1, 0);
    timer_channel_output_shadow_config(htim, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_clear_config(htim, TIMER_CH_1, TIMER_OC_CLEAR_DISABLE);
    timer_channel_output_compare_fast_config(htim, TIMER_CH_1, TIMER_OC_FAST_DISABLE);
    timer_channel_free_complementary_config(htim, TIMER_CH_1, &tim_free_comp_param_struct);
    timer_channel_dead_time_config(htim, TIMER_CH_1, ENABLE);
    timer_channel_composite_pwm_mode_config(htim, TIMER_CH_1, DISABLE);
    timer_output_match_pulse_select(htim, TIMER_CH_1, TIMER_PULSE_OUTPUT_NORMAL);
    
    timer_channel_output_config(htim, TIMER_CH_2, &tim_oc_param_struct);
    timer_channel_output_mode_config(htim, TIMER_CH_2, TIMER_OC_MODE_PWM1); // PWM Mode B
    timer_channel_output_pulse_value_config(htim, TIMER_CH_2, 0);
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_2, 0);
    timer_channel_output_shadow_config(htim, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_clear_config(htim, TIMER_CH_2, TIMER_OC_CLEAR_DISABLE);
    timer_channel_output_compare_fast_config(htim, TIMER_CH_2, TIMER_OC_FAST_DISABLE);
    timer_channel_free_complementary_config(htim, TIMER_CH_2, &tim_free_comp_param_struct);
    timer_channel_dead_time_config(htim, TIMER_CH_2, ENABLE);
    timer_channel_composite_pwm_mode_config(htim, TIMER_CH_2, DISABLE);
    timer_output_match_pulse_select(htim, TIMER_CH_2, TIMER_PULSE_OUTPUT_NORMAL);

    timer_multi_mode_channel_mode_config(htim, TIMER_MCH_0, TIMER_MCH_MODE_COMPLEMENTARY);
    timer_multi_mode_channel_mode_config(htim, TIMER_MCH_1, TIMER_MCH_MODE_COMPLEMENTARY);
    timer_multi_mode_channel_mode_config(htim, TIMER_MCH_2, TIMER_MCH_MODE_COMPLEMENTARY);

    tim_break_param_struct.runoffstate = TIMER_ROS_STATE_ENABLE;
    tim_break_param_struct.ideloffstate = TIMER_IOS_STATE_ENABLE;
    tim_break_param_struct.deadtime = 5;
    tim_break_param_struct.outputautostate = TIMER_OUTAUTO_ENABLE; // controls the automatic recovery of PWM after break signal disappear
    tim_break_param_struct.protectmode = TIMER_CCHP0_PROT_OFF;

    tim_break_param_struct.break0state = TIMER_BREAK0_ENABLE; // Enable Break0 (internal LVD & Hardfault lockup)
    tim_break_param_struct.break0filter = 0; // internal signal bypasses BRK0F
    tim_break_param_struct.break0polarity = TIMER_BREAK0_POLARITY_HIGH; // internal signal bypasses BRK0P
    tim_break_param_struct.break0lock = TIMER_BREAK0_LK_DISABLE;
    tim_break_param_struct.break0release = TIMER_BREAK0_UNRELEASE;

    tim_break_param_struct.break1release = TIMER_BREAK1_UNRELEASE;
    timer_break_config(htim, &tim_break_param_struct);

    timer_dead_time_different_config(htim, DISABLE);

    timer_auto_reload_shadow_enable(htim);

    EnableBridges(Bridge::HB_U, Bridge::HB_V, Bridge::HB_W,
                  Bridge::LB_U, Bridge::LB_V, Bridge::LB_W);

    SetOutput3CHRaw(0, 0, 0);

    if(initCNT)
    {
        EnableAllOutputs();
        timer_enable(htim);
    }
    return FuncRetCode::OK;
}

void FOCDriver6PWM::SetOutput3CHRaw(uint32_t ch1, uint32_t ch2, uint32_t ch3)
{
    // PWM Mode2: max_compare - ch1, PWM Mode1: ch1
    // timer_channel_output_pulse_value_config()
    // timer_channel_output_pulse_fract_value_config() (for DITHERING mode, max_compare * 16)

    // htim->c1dt = max_compare - ch1;
    // htim->c2dt = max_compare - ch2;
    // htim->c3dt = max_compare - ch3;
#if defined(TIM_DITHERING_MODE_EN)
    uint32_t ch_raw = max_compare - ch1;
    timer_channel_output_pulse_value_config(htim, TIMER_CH_0, (uint32_t)(ch_raw >> 4U));
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_0, (uint32_t)(ch_raw & 0xFU));
    ch_raw = max_compare - ch2;
    timer_channel_output_pulse_value_config(htim, TIMER_CH_1, (uint32_t)(ch_raw >> 4U));
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_1, (uint32_t)(ch_raw & 0xFU));
    ch_raw = max_compare - ch3;
    timer_channel_output_pulse_value_config(htim, TIMER_CH_2, (uint32_t)(ch_raw >> 4U));
    timer_channel_output_pulse_fract_value_config(htim, TIMER_CH_2, (uint32_t)(ch_raw & 0xFU));
#else
    timer_channel_output_pulse_value_config(htim, TIMER_CH_0, max_compare - ch1);
    timer_channel_output_pulse_value_config(htim, TIMER_CH_1, max_compare - ch2);
    timer_channel_output_pulse_value_config(htim, TIMER_CH_2, max_compare - ch3);
#endif
}

void FOCDriver6PWM::EnableBridge(Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_CH0EN; break;
        case Bridge::HB_V: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_CH1EN; break;
        case Bridge::HB_W: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_CH2EN; break;
        case Bridge::LB_U: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_MCH0EN; break;
        case Bridge::LB_V: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_MCH1EN; break;
        case Bridge::LB_W: TIMER_CHCTL2(htim) |= (uint32_t)TIMER_CHCTL2_MCH2EN; break;
    }
}

void FOCDriver6PWM::DisableBridge(Bridge bridge)
{
    switch(bridge)
    {
        case Bridge::HB_U: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_CH0EN); break;
        case Bridge::HB_V: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_CH1EN); break;
        case Bridge::HB_W: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_CH2EN); break;
        case Bridge::LB_U: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_MCH0EN); break;
        case Bridge::LB_V: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_MCH1EN); break;
        case Bridge::LB_W: TIMER_CHCTL2(htim) &= (~(uint32_t)TIMER_CHCTL2_MCH2EN); break;
    }
}

real_t FOCDriver6PWM::GetDeadTime()
{
    real_t dead_time = 0.0f;
    const auto timer_base_clock = HAL::GetCoreClockHz();
    float Tdts = 1.0f / (float)timer_base_clock;
    auto DTC = GET_BITS(TIMER_CCHP0(htim), 0, 7);
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