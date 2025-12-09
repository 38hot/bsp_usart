/**
  ******************************************************************************
  * @file           : bsp_usart.h
  * @brief          : Header for bsp_usart.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 kripac@163.com
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H
#define __BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwrb/lwrb.h"
#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define USE_USART1
//#define USE_USART2
//#define USE_USART3

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/* USART1 --------------------------------------------------------------------*/
#ifdef USE_USART1

void USART1_Init(void);
void USART1_Reset();		/* Clear buffer data */

/* Normal mode block mode tranmit */
HAL_StatusTypeDef USART1_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer block mode receive */
HAL_StatusTypeDef USART1_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer data read */
uint16_t USART1_ReadRB(uint8_t *pData, uint16_t MaxSize);

#endif

/* USART2 --------------------------------------------------------------------*/
#ifdef USE_USART2

void USART2_Init(void);
void USART2_Reset();		/* Clear buffer data */

/* Normal mode block mode tranmit */
HAL_StatusTypeDef USART2_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer block mode receive */
HAL_StatusTypeDef USART2_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer data read */
uint16_t USART2_ReadRB(uint8_t *pData, uint16_t MaxSize);

#endif

/* USART3 --------------------------------------------------------------------*/
#ifdef USE_USART3

void USART3_Init(void);
void USART3_Reset();		/* Clear buffer data */

/* Normal mode block mode tranmit */
HAL_StatusTypeDef USART3_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer block mode receive */
HAL_StatusTypeDef USART3_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* Ring buffer data read */
uint16_t USART3_ReadRB(uint8_t *pData, uint16_t MaxSize);

#endif


#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H */



