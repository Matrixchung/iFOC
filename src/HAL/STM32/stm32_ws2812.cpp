#include "stm32_ws2812.hpp"

#define DMA_BUF_OFFSET_HEAD 30
#define DMA_BUF_LEN(x) ((2 * (DMA_BUF_OFFSET_HEAD)) + ((x) * 24) + 1)
#define BIT0_HIGH_TIME (0.250f * 1e-6f) // 0.250us
#define BIT1_HIGH_TIME (0.850f * 1e-6f) // 0.850us

#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

namespace iFOC::HAL
{
WS2812::WS2812(TIM_TypeDef* _htim, uint32_t _ch, DMA_TypeDef* _hdma, uint32_t _dma_ch, uint8_t count) :
                htim(_htim), channel(_ch), hdma(_hdma), dma_channel(_dma_ch), light_count(count)
{
    if(light_count == 0) light_count = 1;
    rgb_buffer.resize(light_count * 3);
    dma_buffer.resize(DMA_BUF_LEN(light_count));
    CCR_address = (uint32_t)(&htim->CCR1);
    TMR_DMA_req_msk = TIM_DIER_CC1DE;
    switch(channel)
    {
        case LL_TIM_CHANNEL_CH2:
        case LL_TIM_CHANNEL_CH2N:
        {
            CCR_address = (uint32_t)(&htim->CCR2);
            TMR_DMA_req_msk = TIM_DIER_CC2DE; break;
        }
        case LL_TIM_CHANNEL_CH3:
        case LL_TIM_CHANNEL_CH3N:
        {
            CCR_address = (uint32_t)(&htim->CCR3);
            TMR_DMA_req_msk = TIM_DIER_CC3DE; break;
        }
        case LL_TIM_CHANNEL_CH4:
        case LL_TIM_CHANNEL_CH4N:
        {
            CCR_address = (uint32_t)(&htim->CCR4);
            TMR_DMA_req_msk = TIM_DIER_CC4DE; break;
        }
        default: break;
    }
}

WS2812::WS2812(TIM_TypeDef* _htim, uint32_t _ch, DMA_TypeDef* _hdma, uint32_t _dma_ch) :
            WS2812(_htim, _ch, _hdma, _dma_ch, 1) {}

FuncRetCode WS2812::Init()
{
    auto tmr_base_clock = GetCoreClockHz();
    uint32_t set_period = tmr_base_clock / 800000;
    if(set_period <= 1) return FuncRetCode::HARDWARE_ERROR;
    set_period -= 1;
    int32_t diff = (int32_t)set_period - (int32_t)LL_TIM_GetAutoReload(htim);
    if(LL_TIM_GetPrescaler(htim) != 0x00 || LL_TIM_GetAutoReload(htim) == 0 || ABS(diff) >= 50)
    {
        LL_TIM_SetPrescaler(htim, 0);
        LL_TIM_SetAutoReload(htim, set_period);
    }
    bit_1 = (uint32_t)(BIT1_HIGH_TIME * (float)tmr_base_clock);
    bit_0 = (uint32_t)(BIT0_HIGH_TIME * (float)tmr_base_clock);
    SetBrightness(brightness);
    Update();
    return FuncRetCode::OK;
}

void WS2812::Update()
{
    for(uint8_t i = 0; i < light_count; i++) SetDMABuf32(i);
    LL_DMA_DisableChannel(hdma, dma_channel);
    LL_DMA_SetMemoryAddress(hdma, dma_channel, (uint32_t)((uint32_t*)dma_buffer.data()));
    LL_DMA_SetPeriphAddress(hdma, dma_channel, (uint32_t)CCR_address);
    LL_DMA_SetDataLength(hdma, dma_channel, DMA_BUF_LEN(light_count));
    LL_DMA_EnableChannel(hdma, dma_channel);
    SET_BIT(htim->DIER, TMR_DMA_req_msk);
    LL_TIM_CC_EnableChannel(htim, channel);
    LL_TIM_EnableAllOutputs(htim);
    LL_TIM_EnableCounter(htim);
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