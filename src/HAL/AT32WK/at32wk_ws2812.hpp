#pragma once

#include "../../Common/Interface/indicator_ws2812_base.hpp"
#include "hal_const.h"
#include "hal_impl.hpp"

#if defined(AT32WK_ENV) && defined(TMR_MODULE_ENABLED) && defined(DMA_MODULE_ENABLED)

namespace iFOC::HAL
{
class WS2812
{
    DELETE_COPY_CONSTRUCTOR(WS2812);
    OVERRIDE_NEW();
public:
    WS2812(tmr_type* _htim, tmr_channel_select_type _ch, dma_channel_type* _hdma, uint8_t count);
    WS2812(tmr_type* _htim, tmr_channel_select_type _ch, dma_channel_type* _hdma);
    FuncRetCode Init();
    void Update();
    void SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    void SetBrightness(float _brightness);
private:
    void SetDMABuf32(uint8_t index);
    tmr_type* htim;
    tmr_channel_select_type channel;
    dma_channel_type* hdma;
    float brightness = 1.0f;
    uint32_t bit_1 = 0;
    uint32_t bit_0 = 0;
    uint32_t CCR_address = 0;
    tmr_dma_request_type dma_request = TMR_C1_DMA_REQUEST;
    Vector<uint8_t> rgb_buffer{};
    Vector<uint32_t> dma_buffer{};
    uint8_t light_count = 1;
};
}

#endif