/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "uart_handler.h"
#include "oled.h"
#include "adc_handler.h"
#include "pwm_encoder.h"
#include "delay.h"
#include "foc.h"
#include "spi_encoder.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC_Z_Pin LL_GPIO_PIN_13
#define ENC_Z_GPIO_Port GPIOC
#define ENC_Z_EXTI_IRQn EXTI15_10_IRQn
#define ENC_B_Pin LL_GPIO_PIN_14
#define ENC_B_GPIO_Port GPIOC
#define ENC_B_EXTI_IRQn EXTI15_10_IRQn
#define ENC_A_Pin LL_GPIO_PIN_15
#define ENC_A_GPIO_Port GPIOC
#define ENC_A_EXTI_IRQn EXTI15_10_IRQn
#define SENSE_BAT_Pin LL_GPIO_PIN_0
#define SENSE_BAT_GPIO_Port GPIOA
#define SENSE_1_Pin LL_GPIO_PIN_2
#define SENSE_1_GPIO_Port GPIOA
#define SENSE_3_Pin LL_GPIO_PIN_3
#define SENSE_3_GPIO_Port GPIOA
#define SPI1_SCK_Pin LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_CS_Pin LL_GPIO_PIN_7
#define SPI1_CS_GPIO_Port GPIOA
#define DRV_IN2_Pin LL_GPIO_PIN_0
#define DRV_IN2_GPIO_Port GPIOB
#define DRV_IN3_Pin LL_GPIO_PIN_1
#define DRV_IN3_GPIO_Port GPIOB
#define DRV_EN_Pin LL_GPIO_PIN_12
#define DRV_EN_GPIO_Port GPIOB
#define PWM_AUX2_Pin LL_GPIO_PIN_8
#define PWM_AUX2_GPIO_Port GPIOA
#define DRV_FAULT_Pin LL_GPIO_PIN_11
#define DRV_FAULT_GPIO_Port GPIOA
#define DRV_FAULT_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin LL_GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define KEY_Pin LL_GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define DRV_IN1_Pin LL_GPIO_PIN_4
#define DRV_IN1_GPIO_Port GPIOB
#define OLED_SCL_Pin LL_GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin LL_GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
