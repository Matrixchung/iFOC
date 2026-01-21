#pragma once

#include "../../Common/Interface/indicator_rgb_base.hpp"
#include "hal_const.h"
#include "hal_impl.hpp"

#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

namespace iFOC::HAL
{
class WS2812 final : public IndicatorRGB
{
    DELETE_COPY_CONSTRUCTOR(WS2812);
    OVERRIDE_NEW();
public:
    WS2812(TIM_TypeDef* _htim, uint32_t _ch, DMA_TypeDef* _hdma, uint32_t _dma_ch, uint8_t count);
    WS2812(TIM_TypeDef* _htim, uint32_t _ch, DMA_TypeDef* _hdma, uint32_t _dma_ch);
    FuncRetCode Init() override;
    void Update() override;
    void SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) override;
    void SetBrightness(float _brightness) override;
private:
    void SetDMABuf32(uint8_t index);
    TIM_TypeDef* htim;
    uint32_t channel;
    DMA_TypeDef* hdma;
    uint32_t dma_channel;
    float brightness = 1.0f;
    uint32_t bit_1 = 0;
    uint32_t bit_0 = 0;
    uint32_t CCR_address = 0;
    uint32_t TMR_DMA_req_msk = 0;
    Vector<uint8_t> rgb_buffer{};
    Vector<uint32_t> dma_buffer{};
    uint8_t light_count = 1;
};
}

#endif