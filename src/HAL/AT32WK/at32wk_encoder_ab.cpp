#include "at32wk_encoder_ab.hpp"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED)

namespace iFOC::Encoder
{
EncoderAB::EncoderAB(tmr_type* _htim, uint32_t _cpr) : EncoderABBase(_cpr), htim(_htim) {}

FuncRetCode EncoderAB::Init(uint8_t motor_id)
{
    tmr_reset(htim);

    tmr_cnt_dir_set(htim, TMR_COUNT_UP);
    tmr_clock_source_div_set(htim, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(htim, TRUE);
    tmr_base_init(htim, cpr, 0);

    tmr_sub_sync_mode_set(htim, FALSE);
    tmr_primary_mode_select(htim, TMR_PRIMARY_SEL_RESET);

    tmr_input_config_type tmr_input_struct
    {
        .input_channel_select = TMR_SELECT_CHANNEL_1,
        .input_polarity_select = TMR_INPUT_RISING_EDGE,
        .input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT,
        .input_filter_value = 1
    };

    tmr_input_channel_init(htim, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

    tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;

    tmr_input_channel_init(htim, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

    tmr_encoder_mode_config(htim, TMR_ENCODER_MODE_C, TMR_INPUT_RISING_EDGE, TMR_INPUT_RISING_EDGE);

    tmr_interrupt_enable(htim, TMR_OVF_INT | TMR_C1_INT | TMR_C2_INT | TMR_C3_INT | TMR_C4_INT | TMR_HALL_INT | TMR_TRIGGER_INT | TMR_BRK_INT, FALSE);

    tmr_counter_enable(htim, TRUE);

    return FuncRetCode::OK;
}

void EncoderAB::UpdatePulse()
{
    pulse = (short)(tmr_counter_value_get(htim));
}
}

#endif