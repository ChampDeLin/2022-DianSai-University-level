/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
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

#define PC_RX_DMA_MAXSIZE 100

typedef struct pc_t {
    uint8_t rx_dma_buffer[PC_RX_DMA_MAXSIZE];
    uint8_t rx_dma_buffer_shadow[PC_RX_DMA_MAXSIZE];///< DMA接收影子数组
    
    UART_HandleTypeDef *huart;

	volatile data_state_t rev_state;
    volatile uint8_t rev_write_index;
    volatile uint8_t rev_buffer[PC_RX_DMA_MAXSIZE + 1];
}pc_t;
extern pc_t g_pc;

void pc__rev_msg(uint8_t *revbuff, uint8_t revlength);

void beep_on(void);
void beep_off(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BEEP_Pin GPIO_PIN_15
#define BEEP_GPIO_Port GPIOC
#define oled_scl_Pin GPIO_PIN_8
#define oled_scl_GPIO_Port GPIOB
#define oled_sda_Pin GPIO_PIN_9
#define oled_sda_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
