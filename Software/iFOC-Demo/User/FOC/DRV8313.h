#ifndef _DRV8313_H
#define _DRV8313_H

#include "main.h"
#include "gpio.h"

#define DRV8313_FAULT_IRQn DRV_FAULT_EXTI_IRQn
#define DRV8313_FAULT_PIN_PORT DRV_FAULT_GPIO_Port
#define DRV8313_FAULT_PIN DRV_FAULT_Pin
#define DRV8313_EN_PIN_PORT DRV_EN_GPIO_Port
#define DRV8313_EN_PIN DRV_EN_Pin

#define DRV8313_FULL_PWM 1000

#define _constrain(v,low,high) ((v)<(low)?(low):((v)>(high)?(high):(v)))

void DRV8313_PWM_Init(void);
void DRV8313_FAULT_IRQHandler(void);
void DRV8313_SetState(uint8_t state);
void DRV8313_SetPWMPercent(float pct_a, float pct_b, float pct_c);
void DRV8313_ResetPWM(void);
#endif