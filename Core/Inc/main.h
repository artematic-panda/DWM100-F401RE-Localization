/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f401xe.h"
#include "stm32f4xx_ll_spi.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {  // 40-bit timestamp type for DWM1000 timestamps
  uint32_t  BOT4; // must be first declaration
  uint8_t   TOP1;
} TIME;

typedef float COORD[3]; // x, y, z; gimbal expects unit vectors
// ^^^^ this was honestly a poor move because we keep casting to a float in the code
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// UWB networking config definition
#define TAG_ADDR     (0x3730 + 0x0)
#define GIM_ADDR     (0x3730 + 0x1)
#define SCN_ADDR     (0x3730 + 0x2)
#define AN3_ADDR     (0x3730 + 0x3)
#define AN4_ADDR     (0x3730 + 0x4)
#define AN5_ADDR     (0x3730 + 0x5)

#define TAG           TAG_ADDR
#define GIM           GIM_ADDR
#define SCN           SCN_ADDR
#define AN3           AN3_ADDR
#define AN4           AN4_ADDR
#define AN5           AN5_ADDR

#define CONFIG        TAG            // options: {TAG, GIM, SCN, AN3, AN4, AN5}

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LASER_Pin GPIO_PIN_4
#define LASER_GPIO_Port GPIOA
#define DWM_SPI1_SCK_Pin GPIO_PIN_5
#define DWM_SPI1_SCK_GPIO_Port GPIOA
#define DWM_SPI1_MISO_Pin GPIO_PIN_6
#define DWM_SPI1_MISO_GPIO_Port GPIOA
#define DWM_SPI1_MOSI_Pin GPIO_PIN_7
#define DWM_SPI1_MOSI_GPIO_Port GPIOA
#define DWM_WAKE_Pin GPIO_PIN_7
#define DWM_WAKE_GPIO_Port GPIOC
#define DWM_RESET_Pin GPIO_PIN_8
#define DWM_RESET_GPIO_Port GPIOA
#define DWM_IRQ_Pin GPIO_PIN_9
#define DWM_IRQ_GPIO_Port GPIOA
#define DWM_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DWM_CS_Pin GPIO_PIN_6
#define DWM_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TRUE                1
#define FALSE               0

// #define EXTENDED_FRAMES                  // extended frame length support
#define DWM_DEVICE_ID       0xDECA0130U
#define PAN_ID              0xE373          // Network ID
#if   CONFIG == TAG
#define SHORT_ADDR          TAG_ADDR        // Device address on network
#elif CONFIG == GIM       
#define SHORT_ADDR          GIM_ADDR        // Device address on network
#elif CONFIG == SCN      
#define SHORT_ADDR          SCN_ADDR        // Device address on network
#elif CONFIG == AN3      
#define SHORT_ADDR          AN3_ADDR        // Device address on network
#elif CONFIG == AN4       
#define SHORT_ADDR          AN4_ADDR        // Device address on network
#elif CONFIG == AN5      
#define SHORT_ADDR          AN5_ADDR        // Device address on network
#endif

// Packet loss defines
#define TX_WATCHDOG         TIM1
#define TX_TIMEOUT          400U - 1      // uS (largely depends on preamble elected)

// misc.
// #define SPI_TIMEOUT_DELAY   100U          // ms
#define ADJ_SPEED_OF_LIGHT  0.02997924 * (16/15.65004006) * 8 * 1.885797   // speed of light in cm/ps with adjustment for bit-shifts in 15.65ps -> 1ps conversion and one-way from round-trip; + experimental adjustment
#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#define MAX_SAMPLE_DELTA    3E1              // cm
#define DATA_SEND_RETRIES   100U             // init anchor(in master state) downgrade-to-slave safety
#define GIMBAL_MOVE_RETRIES 10U              // normal operation retry sending current coordinate; if this expires, calculating a new coordinate is preferred for accuracy

// Frame defines
#define BASE_FRAME_SIZE     9U               // MAC base frame, without data
#define D_ADDR_OFFSET       5U               // destination address
#define S_ADDR_OFFSET       7U               // source address


// verified nodes defines
#define GIM_UNVERIFIED      0x0U
#define SCN_UNVERIFIED      0x1U
#define AN3_UNVERIFIED      0x2U
#define AN4_UNVERIFIED      0x3U
#define AN5_UNVERIFIED      0x4U
#define ALL_NODES_VERIFIED  0x5U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
