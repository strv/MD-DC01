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
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_dac.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_i2c.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
  int32_t v_ff_kp_q8;   // Feed forward P gain
  int32_t v_kp_q8;      // Feed back P gain
  int32_t v_ti_q8;      // Feed back I gain
  int32_t brush_co_mv;  // Brush compensation voltage
  int32_t max_mv;       // Maximum output voltage
  int32_t max_cur;      // Maximum output current
} CtrlConfs;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
const static int32_t CtrlQ = 8;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void adc_cb();
void pwm_cb();
void usart_rx_cb(void);
void usart_tx_cb(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_CYCLE ((PWM_CLOCK / PWM_HZ) - 1)
#define COM_BAUD (115200)
#define PWM_HZ (100*1000)
#define PWM_CLOCK (64*1000*1000)
#define LED2_Pin LL_GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin LL_GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define USER_SW_Pin LL_GPIO_PIN_2
#define USER_SW_GPIO_Port GPIOF
#define ADC_VM_Pin LL_GPIO_PIN_0
#define ADC_VM_GPIO_Port GPIOA
#define ADC_CUR_Pin LL_GPIO_PIN_1
#define ADC_CUR_GPIO_Port GPIOA
#define ADC_BEMF_A_Pin LL_GPIO_PIN_2
#define ADC_BEMF_A_GPIO_Port GPIOA
#define ADC_BEMF_B_Pin LL_GPIO_PIN_3
#define ADC_BEMF_B_GPIO_Port GPIOA
#define ADC_AUX0_Pin LL_GPIO_PIN_6
#define ADC_AUX0_GPIO_Port GPIOA
#define ADC_AUX1_Pin LL_GPIO_PIN_7
#define ADC_AUX1_GPIO_Port GPIOA
#define GPIO_AUX0_Pin LL_GPIO_PIN_0
#define GPIO_AUX0_GPIO_Port GPIOB
#define GPIO_AUX1_Pin LL_GPIO_PIN_1
#define GPIO_AUX1_GPIO_Port GPIOB
#define TP4_Pin LL_GPIO_PIN_2
#define TP4_GPIO_Port GPIOB
#define PWM_A_Pin LL_GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin LL_GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define BEMF_GAIN_Pin LL_GPIO_PIN_6
#define BEMF_GAIN_GPIO_Port GPIOC
#define LED0_Pin LL_GPIO_PIN_10
#define LED0_GPIO_Port GPIOA
#define LED1_Pin LL_GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define TP5_Pin LL_GPIO_PIN_12
#define TP5_GPIO_Port GPIOA
#define TP6_Pin LL_GPIO_PIN_15
#define TP6_GPIO_Port GPIOA
#define ENC_Z_Pin LL_GPIO_PIN_3
#define ENC_Z_GPIO_Port GPIOB
#define ENC_A_Pin LL_GPIO_PIN_4
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin LL_GPIO_PIN_5
#define ENC_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PwmTim TIM1
#define UART USART1
#define UART_DMA_TX_CH LL_DMA_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
