#ifndef _DRIVER_DRV8323_H
#define _DRIVER_DRV8323_H

#include "driver_drv8323_base.hpp"
#include "global_include.h"

typedef struct drv8323_pin_t
{
    GPIO_TypeDef *CS_Port;
    uint32_t CS_Pin;
    GPIO_TypeDef *EN_Port;
    uint32_t EN_Pin;
    GPIO_TypeDef *nFAULT_Port;
    uint32_t nFAULT_Pin;
}drv8323_pin_t;

class STM32_DRV8323 : public DriverDRV8323Base<STM32_DRV8323>
{
public:
    STM32_DRV8323(TIM_TypeDef *_htim, SPI_TypeDef *_hspi, const uint16_t _reg_hs, const uint16_t _reg_ls, drv8323_pin_t& pin_t)
    : DriverDRV8323Base(_reg_hs, _reg_ls), htim(_htim), hspi(_hspi)
    {
        CS_Port = pin_t.CS_Port;
        CS_Pin = pin_t.CS_Pin;
        EN_Port = pin_t.EN_Port;
        EN_Pin = pin_t.EN_Pin;
        nFAULT_Port = pin_t.nFAULT_Port;
        nFAULT_Pin = pin_t.nFAULT_Pin;
    };
    // bool IsEnabledOutput();
    void EnableOutput();
    void DisableOutput();
    void SetLSIdleState(uint8_t state);
    bool PortSPIInit();
    bool PortTIMInit();
    uint16_t PortSPIRead16(uint16_t reg, uint16_t *data);
    void PortSetEN(uint8_t state);
    void PortSetCS(uint8_t state);
    bool PortReadFAULT();
    void PortDelayUs(uint32_t us);
    void PortSetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3);
private:
    TIM_TypeDef *htim = nullptr;
    SPI_TypeDef *hspi = nullptr;
    GPIO_TypeDef *CS_Port = nullptr;
    uint32_t CS_Pin;
    GPIO_TypeDef *EN_Port = nullptr;
    uint32_t EN_Pin;
    GPIO_TypeDef *nFAULT_Port = nullptr;
    uint32_t nFAULT_Pin;
};

// bool STM32_DRV8323::IsEnabledOutput()
// {
//     return (htim->BDTR & TIM_BDTR_MOE_Msk);
// }

void STM32_DRV8323::EnableOutput()
{
    LL_TIM_EnableAllOutputs(htim);
}

void STM32_DRV8323::DisableOutput()
{
    LL_TIM_DisableAllOutputs(htim);
}

void STM32_DRV8323::SetLSIdleState(uint8_t state)
{
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH1N, state == 0 ? LL_TIM_OCIDLESTATE_LOW : LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH2N, state == 0 ? LL_TIM_OCIDLESTATE_LOW : LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH3N, state == 0 ? LL_TIM_OCIDLESTATE_LOW : LL_TIM_OCIDLESTATE_HIGH);
}

bool STM32_DRV8323::PortSPIInit()
{
    if(hspi == nullptr || CS_Port == nullptr || EN_Port == nullptr || nFAULT_Port == nullptr) return false;
    LL_SPI_Enable(hspi);
    LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    return true;
}

bool STM32_DRV8323::PortTIMInit()
{
    if(htim == nullptr || CS_Port == nullptr || EN_Port == nullptr || nFAULT_Port == nullptr) return false;
    max_compare = LL_TIM_GetAutoReload(htim);
    half_compare = max_compare / 2u;
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH3N);
    LL_TIM_CC_EnableChannel(htim, LL_TIM_CHANNEL_CH4);
    __attribute__((unused)) uint32_t tmpsmcr = htim->SMCR & TIM_SMCR_SMS;
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH3, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH1N, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH2N, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetIdleState(htim, LL_TIM_CHANNEL_CH3N, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_EnableAllOutputs(htim);
    LL_TIM_EnableCounter(htim);
    return true;
}

uint16_t STM32_DRV8323::PortSPIRead16(uint16_t reg, uint16_t *data)
{
    return SPI_ReadReg16(hspi, reg, data);
}

void STM32_DRV8323::PortSetEN(uint8_t state)
{
    if(state) LL_GPIO_SetOutputPin(EN_Port, EN_Pin);
    else LL_GPIO_ResetOutputPin(EN_Port, EN_Pin);
}

void STM32_DRV8323::PortSetCS(uint8_t state)
{
    if(state) LL_GPIO_SetOutputPin(CS_Port, CS_Pin);
    else LL_GPIO_ResetOutputPin(CS_Port, CS_Pin);
}

bool STM32_DRV8323::PortReadFAULT()
{
    return LL_GPIO_IsInputPinSet(nFAULT_Port, nFAULT_Pin);
}

void STM32_DRV8323::PortDelayUs(uint32_t us)
{
    delay_us(us);
}

void STM32_DRV8323::PortSetOutputRaw(uint16_t ch_1, uint16_t ch_2, uint16_t ch_3)
{
    LL_TIM_OC_SetCompareCH1(htim, ch_1);
    LL_TIM_OC_SetCompareCH2(htim, ch_2);
    LL_TIM_OC_SetCompareCH3(htim, ch_3);
}
#endif