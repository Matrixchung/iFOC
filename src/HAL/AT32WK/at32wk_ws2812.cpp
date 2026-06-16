#include "at32wk_ws2812.hpp"

#define DMA_BUF_OFFSET_HEAD 30
#define DMA_BUF_LEN(x) ((2 * (DMA_BUF_OFFSET_HEAD)) + ((x) * 24) + 1)
#define BIT0_HIGH_TIME (0.250f * 1e-6f) // 0.250us
#define BIT1_HIGH_TIME (0.850f * 1e-6f) // 0.850us

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED) && defined(DMA_MODULE_ENABLED)

namespace iFOC::HAL
{
WS2812::WS2812(tmr_type *_htim, tmr_channel_select_type _ch, dma_channel_type *_hdma, uint8_t count) :
                htim(_htim), channel(_ch), hdma(_hdma), light_count(count)
{
    if(light_count == 0) light_count = 1;
    rgb_buffer.resize(light_count * 3);
    dma_buffer.resize(DMA_BUF_LEN(light_count));
    CCR_address = (uint32_t)(&htim->c1dt);
    dma_request = TMR_C1_DMA_REQUEST;
    switch(channel)
    {
        case TMR_SELECT_CHANNEL_2:
        case TMR_SELECT_CHANNEL_2C:
        {
            dma_request = TMR_C2_DMA_REQUEST;
            CCR_address = (uint32_t)(&htim->c2dt); break;
        }
        case TMR_SELECT_CHANNEL_3:
        case TMR_SELECT_CHANNEL_3C:
        {
            dma_request = TMR_C3_DMA_REQUEST;
            CCR_address = (uint32_t)(&htim->c3dt); break;
        }
        case TMR_SELECT_CHANNEL_4:
        {
            dma_request = TMR_C4_DMA_REQUEST;
            CCR_address = (uint32_t)(&htim->c4dt); break;
        }
        default: break;
    }
}

WS2812::WS2812(tmr_type *_htim, tmr_channel_select_type _ch, dma_channel_type *_hdma) :
                WS2812(_htim, _ch, _hdma, 1) {}

FuncRetCode WS2812::Init()
{
    tmr_reset(htim);
    const auto tmr_base_clock = GetCoreClockHz();
    uint32_t set_period = tmr_base_clock / 800000;
    if(set_period <= 1) return FuncRetCode::HARDWARE_ERROR;
    set_period -= 1;
    int32_t diff = (int32_t)set_period - (int32_t)tmr_period_value_get(htim);
    if(tmr_div_value_get(htim) != 0x00 ||
        tmr_period_value_get(htim) == 0 || tmr_period_value_get(htim) == 65535 ||
        ABS(diff) >= 50) // wrong DIVider & period
    {
        tmr_div_value_set(htim, 0x00);
        tmr_period_value_set(htim, set_period);
    }
    bit_1 = (uint32_t)(BIT1_HIGH_TIME * (float)tmr_base_clock);
    bit_0 = (uint32_t)(BIT0_HIGH_TIME * (float)tmr_base_clock);
    // Enabling buffers, which is mandatory for AT32xx. If not, output will be messed up
    tmr_period_buffer_enable(htim, TRUE);
    tmr_channel_buffer_enable(htim, TRUE);
    tmr_output_channel_buffer_enable(htim, channel, TRUE);
    tmr_overflow_event_disable(htim, FALSE); // ENABLE OVERFLOW EVENT!

    // Reset channel to PWM MODE A
    tmr_output_config_type tmr_output_struct;
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;
    tmr_output_struct.occ_output_state = FALSE;
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;
    tmr_output_struct.occ_idle_state = FALSE;
    tmr_output_channel_config(htim, channel, &tmr_output_struct);
    tmr_channel_value_set(htim, channel, 0);
    tmr_output_channel_buffer_enable(htim, channel, TRUE);
    tmr_output_channel_immediately_set(htim, channel, FALSE);

    hdma->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_WORD;
    hdma->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    hdma->ctrl_bit.mincm = TRUE;  // Memory Increment Enable
    hdma->ctrl_bit.pincm = FALSE; // Periph. Increment Disable
#if defined(AT32F403Axx) || defined(AT32F435xx)
    hdma->ctrl_bit.lm = TRUE; // Circular mode
#endif
    SetBrightness(brightness);
    Update();
    return FuncRetCode::OK;
}

void WS2812::Update()
{
    dma_channel_enable(hdma, FALSE);
    for(uint8_t i = 0; i < light_count; i++) SetDMABuf32(i);
    hdma->maddr = (uint32_t)((uint32_t*)dma_buffer.data());
    hdma->paddr = (uint32_t)CCR_address;
    hdma->dtcnt = DMA_BUF_LEN(light_count);
    dma_channel_enable(hdma, TRUE);
    // tmr_dma_request_enable(htim, dma_request, TRUE);
    htim->iden |= dma_request;
    tmr_channel_enable(htim, channel, TRUE);
    tmr_output_enable(htim, TRUE);
    tmr_counter_enable(htim, TRUE);
}

void WS2812::SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index >= light_count) index = light_count - 1;
    rgb_buffer[index * 3 + 0] = (uint8_t)((float)g * brightness);
    rgb_buffer[index * 3 + 1] = (uint8_t)((float)r * brightness);
    rgb_buffer[index * 3 + 2] = (uint8_t)((float)b * brightness);
}

void WS2812::SetBrightness(float _brightness)
{
    brightness = _constrain(_brightness, 0.0f, 1.0f);
}

void WS2812::SetDMABuf32(uint8_t index)
{
    if(index >= light_count) index = light_count - 1;
    for(uint8_t i = 0; i < 8; i++)
    {
        *(dma_buffer.data() + DMA_BUF_OFFSET_HEAD + 24 * index + i) = (rgb_buffer[index * 3 + 0] & (0x80 >> i)) ? bit_1 : bit_0;
        *(dma_buffer.data() + DMA_BUF_OFFSET_HEAD + 24 * index + i + 8) = (rgb_buffer[index * 3 + 1] & (0x80 >> i)) ? bit_1 : bit_0;
        *(dma_buffer.data() + DMA_BUF_OFFSET_HEAD + 24 * index + i + 16) = (rgb_buffer[index * 3 + 2] & (0x80 >> i)) ? bit_1 : bit_0;
    }
}
}

#endif
