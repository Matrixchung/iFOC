#include "DRV8313.h"
uint8_t drv8313_fault = 0;
uint8_t drv8313_state = 0;

void DRV8313_PWM_Init(void)
{
    // PWM Frequency above 25KHz reduces noise
    // PWMFreq = 25KHz
    // Prescaler = 9-1, Counter Period (ARR) = 320-1 (full 320) (for more precise control)
    // PWMFreq = 30KHz
    // Prescaler = 4-1, ARR = 600-1
    // PWMFreq = 36KHz
    // Prescaler = 2-1, ARR = 1000-1

    // HAL Version
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Timer 3 CH1 - DRV_IN1
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Timer 3 CH3 - DRV_IN2
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Timer 3 CH4 - DRV_IN3
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    // LL Version
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_OC_SetCompareCH3(TIM3, 0);
    LL_TIM_OC_SetCompareCH4(TIM3, 0);
}
// call in EXIT_IRQHandler()
void DRV8313_FAULT_IRQHandler(void)
{
    // interrupt triggered, then read pinstate
    drv8313_fault = (LL_GPIO_IsInputPinSet(DRV8313_FAULT_PIN_PORT, DRV8313_FAULT_PIN) == 0);
    if(drv8313_fault) DRV8313_SetState(0); // shut down motor driver
}

void DRV8313_SetState(uint8_t state)
{
    drv8313_state = (state != 0);
    if(drv8313_state)
    {
        LL_GPIO_SetOutputPin(DRV8313_EN_PIN_PORT, DRV8313_EN_PIN);
    }
    else LL_GPIO_ResetOutputPin(DRV8313_EN_PIN_PORT, DRV8313_EN_PIN);
}

void DRV8313_SetPWMPercent(float pct_a, float pct_b, float pct_c)
{
    pct_a = _constrain(pct_a, 0.0f, 1.0f);
    pct_b = _constrain(pct_b, 0.0f, 1.0f);
    pct_c = _constrain(pct_c, 0.0f, 1.0f);
    // HAL Version
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(pct_a * DRV8313_FULL_PWM));
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)(pct_b * DRV8313_FULL_PWM));
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(pct_c * DRV8313_FULL_PWM));

    // LL Version
    LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)(pct_a * DRV8313_FULL_PWM));
    LL_TIM_OC_SetCompareCH3(TIM3, (uint32_t)(pct_b * DRV8313_FULL_PWM));
    LL_TIM_OC_SetCompareCH4(TIM3, (uint32_t)(pct_c * DRV8313_FULL_PWM));
}

void DRV8313_ResetPWM(void)
{
    DRV8313_SetPWMPercent(0, 0, 0);
}