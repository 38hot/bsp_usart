/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @brief   This file provides code for usart operation
			 bsp_usart V1.0, 2025/4/1
			 bsp_usart v1.1, 2025/11/24
				1, Add CM7 DCache support
				2, Change default DMA size from 16 to 32
										

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

/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private defines -----------------------------------------------------------*/
#define CACHE_SUPPORT

/* USART1 --------------------------------------------------------------------*/
#ifdef USE_USART1

#define UART1_RX_DMA_BUF_LEN	(32u)
#define UART1_RX_RB_LEN			(129u)			// Recommend: 2^n + 1 bytes


extern osSemaphoreId_t	Usart1RxSemHandle;

/* DMA buffer */
uint8_t	usart1_rx_dma_buf[UART1_RX_DMA_BUF_LEN] __attribute__((aligned(32)));

/* LwRB instance and buffer */
lwrb_t	usart1_rx_rb;							// Ring buffer instance for RX data
uint8_t	usart1_rx_rb_data[UART1_RX_RB_LEN];		// Ring buffer data array for RX DMA


/**
  * @brief  Usr defined Rx Event Callback
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
void USART1_RxEventCb(UART_HandleTypeDef *huart, uint16_t size)
{
	UNUSED(size);	
	static uint16_t	pos_last;
	uint16_t		pos;
	
	#ifdef CACHE_SUPPORT
	/* Invalidate DCache for CM7 core */
	SCB_InvalidateDCache_by_Addr((uint32_t *)&usart1_rx_dma_buf, sizeof(usart1_rx_dma_buf));
	#endif
	
	pos = sizeof(usart1_rx_dma_buf) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (pos != pos_last)
	{
		if (pos > pos_last)
		{	/* Current position is over previous one */
			/*
			 * Processing is done in "linear" mode.
			 *
			 * Application processing is fast with single data block,
			 * length is simply calculated by subtracting pointers
			 *
			 * [   0   ]
			 * [   1   ] <- old_pos |------------------------------------|
			 * [   2   ]            |                                    |
			 * [   3   ]            | Single block (len = pos - old_pos) |
			 * [   4   ]            |                                    |
			 * [   5   ]            |------------------------------------|
			 * [   6   ] <- pos
			 * [   7   ]
			 * [ N - 1 ]
			 */
			if (lwrb_write(&usart1_rx_rb, &usart1_rx_dma_buf[pos_last], pos - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart1 lwrb write fail!\r\n");
				#endif
			}
		}
		else if (pos < pos_last)
		{
			/* "overflow" mode..
			 *
			 * Application must process data twice,
			 * since there are 2 linear memory blocks to handle
			 *
			 * [   0   ]            |---------------------------------|
			 * [   1   ]            | Second block (len = pos)        |
			 * [   2   ]            |---------------------------------|
			 * [   3   ] <- pos
			 * [   4   ] <- old_pos |---------------------------------|
			 * [   5   ]            |                                 |
			 * [   6   ]            | First block (len = N - old_pos) |
			 * [   7   ]            |                                 |
			 * [ N - 1 ]            |---------------------------------|
			 */
			if (lwrb_write(&usart1_rx_rb, &usart1_rx_dma_buf[pos_last], sizeof(usart1_rx_dma_buf) - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart1 lwrb write fail!\r\n");
				#endif
			}
			
			if (pos > 0)	/* Second block process */
			{
				if (lwrb_write(&usart1_rx_rb, &usart1_rx_dma_buf[0], pos) == NULL)
				{
					#ifdef __ENABLE_SHELL
					printf("usart1 lwrb write fail!\r\n");
					#endif
				}
			}
		}
		pos_last = pos;		/* Save current position as old for next transfers */
	}	/* if (pos != pos_last) */
	
	switch (huart->RxEventType)
	{
		case HAL_UART_RXEVENT_IDLE:
			osSemaphoreRelease(Usart1RxSemHandle);
			break;		
		
		case HAL_UART_RXEVENT_TC: 
			break;
		
		case HAL_UART_RXEVENT_HT:
			break;
		
		default:
			#ifdef __ENABLE_SHELL
			printf("RxEventType Error!\r\n");
			#endif
			break;
	}
}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void USART1_ErrorCb(UART_HandleTypeDef *huart)
{
	#ifdef __ENABLE_SHELL
	printf("USART1 Error!\r\n");
	#endif
	
	if (huart == &huart1)
	{
		/* Uart1 connected to master serial port */

		/* Clear error flag */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_CMF);		//Character Match Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);		//IDLE line detected Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);		//Overrun Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);		//Framing Error Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);		//Parity Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);		//Noise detected Clear Flag
		
		
		//__HAL_UNLOCK(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, usart1_rx_dma_buf, sizeof(usart1_rx_dma_buf));
	}
	else
	{
		#ifdef __ENABLE_SHELL
		printf("Other USART1 Error!\r\n");
		#endif
	}
}

/**
  * @brief  UART1 Init
  * @param  None
  * @retval None
  *			Uart should config to DMA Rx circular mode and enable interrupt
  */
void USART1_Init(void)
{
	/* Init LwRB ring fifo */
	lwrb_init(&usart1_rx_rb, usart1_rx_rb_data, sizeof(usart1_rx_rb_data));
	
	/* Register rx event call back */
	HAL_UART_RegisterRxEventCallback(&huart1, USART1_RxEventCb);
	
	/* Register error call back */
	HAL_UART_RegisterCallback(&huart1, HAL_UART_ERROR_CB_ID, USART1_ErrorCb);
	
	/* Start UART */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_dma_buf, sizeof(usart1_rx_dma_buf));
	
	/* Disable error interrupt to prevent unexpected crashes*/
	#if 1
	ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
	//ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
	#endif
}

/**
  * @brief  Block mode receive N byte until timeout
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
HAL_StatusTypeDef USART1_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint32_t tick_start;
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
	
    /* Init tickstart for timeout management */
    tick_start = HAL_GetTick();
	while (lwrb_get_full(&usart1_rx_rb) < Size)
	{
		if (((HAL_GetTick() - tick_start) > Timeout) || (Timeout == 0U))
		{
			return HAL_TIMEOUT;
		}
	}
	
	lwrb_read(&usart1_rx_rb, pData, Size);
	return HAL_OK;
}

/**
  * @brief  Receive data from Ring Buffer
  * @param  Received data
  * @param	Max data size
  * @retval Actual received data length
  */
uint16_t USART1_ReadRB(uint8_t *pData, uint16_t MaxSize)
{
    if ((pData == NULL) || (MaxSize == 0U))
    {
      return  0u;
    }
	uint16_t RecvSize = lwrb_get_full(&usart1_rx_rb);
	uint16_t Size = RecvSize < MaxSize ? RecvSize : MaxSize;
	lwrb_read(&usart1_rx_rb, pData, Size);
	return Size;
}

/**
  * @brief  Clear ring buffer data
  * @param  None
  * @retval None
  */
void USART1_Reset()
{
	/* Reset ring buffer */
	lwrb_reset(&usart1_rx_rb);
}


HAL_StatusTypeDef USART1_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	return HAL_UART_Transmit(&huart1, pData, Size, Timeout);
}

#endif

/* USART2 --------------------------------------------------------------------*/
#ifdef USE_USART2

#define UART2_RX_DMA_BUF_LEN	(32u)
#define UART2_RX_RB_LEN			(129u)


extern osSemaphoreId_t	Usart2RxSemHandle;

/* DMA buffer */
uint8_t	usart2_rx_dma_buf[UART2_RX_DMA_BUF_LEN] __attribute__((aligned(32)));

/* LwRB instance and buffer */
lwrb_t	usart2_rx_rb;							// Ring buffer instance for RX data
uint8_t	usart2_rx_rb_data[UART2_RX_RB_LEN];		// Ring buffer data array for RX DMA


/**
  * @brief  Usr defined Rx Event Callback
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
void USART2_RxEventCb(UART_HandleTypeDef *huart, uint16_t size)
{
	UNUSED(size);	
	static uint16_t	pos_last;
	uint16_t		pos;
	
	#ifdef CACHE_SUPPORT
	/* Invalidate DCache for CM7 core */
	SCB_InvalidateDCache_by_Addr((uint32_t *)&usart2_rx_dma_buf, sizeof(usart2_rx_dma_buf));
	#endif
	
	pos = sizeof(usart2_rx_dma_buf) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (pos != pos_last)
	{
		if (pos > pos_last)
		{	/* Current position is over previous one */
			/*
			 * Processing is done in "linear" mode.
			 *
			 * Application processing is fast with single data block,
			 * length is simply calculated by subtracting pointers
			 *
			 * [   0   ]
			 * [   1   ] <- old_pos |------------------------------------|
			 * [   2   ]            |                                    |
			 * [   3   ]            | Single block (len = pos - old_pos) |
			 * [   4   ]            |                                    |
			 * [   5   ]            |------------------------------------|
			 * [   6   ] <- pos
			 * [   7   ]
			 * [ N - 1 ]
			 */
			if (lwrb_write(&usart2_rx_rb, &usart2_rx_dma_buf[pos_last], pos - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart2 lwrb write fail!\r\n");
				#endif
			}
		}
		else if (pos < pos_last)
		{
			/* "overflow" mode..
			 *
			 * Application must process data twice,
			 * since there are 2 linear memory blocks to handle
			 *
			 * [   0   ]            |---------------------------------|
			 * [   1   ]            | Second block (len = pos)        |
			 * [   2   ]            |---------------------------------|
			 * [   3   ] <- pos
			 * [   4   ] <- old_pos |---------------------------------|
			 * [   5   ]            |                                 |
			 * [   6   ]            | First block (len = N - old_pos) |
			 * [   7   ]            |                                 |
			 * [ N - 1 ]            |---------------------------------|
			 */
			if (lwrb_write(&usart2_rx_rb, &usart2_rx_dma_buf[pos_last], sizeof(usart2_rx_dma_buf) - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart2 lwrb write fail!\r\n");
				#endif
			}
			
			if (pos > 0)	/* Second block process */
			{
				if (lwrb_write(&usart2_rx_rb, &usart2_rx_dma_buf[0], pos) == NULL)
				{
					#ifdef __ENABLE_SHELL
					printf("usart2 lwrb write fail!\r\n");
					#endif
				}
			}
		}
		pos_last = pos;		/* Save current position as old for next transfers */
	}	/* if (pos != pos_last) */
	
	switch (huart->RxEventType)
	{
		case HAL_UART_RXEVENT_IDLE:
			osSemaphoreRelease(Usart2RxSemHandle);
			break;		
		
		case HAL_UART_RXEVENT_TC: 
			break;
		
		case HAL_UART_RXEVENT_HT:
			break;
		
		default:
			#ifdef __ENABLE_SHELL
			printf("RxEventType Error!\r\n");
			#endif
			break;
	}
}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void USART2_ErrorCb(UART_HandleTypeDef *huart)
{
	#ifdef __ENABLE_SHELL
	printf("USART2 Error!\r\n");
	#endif
	
	if (huart == &huart2)
	{
		/* Uart2 connected to master serial port */

		/* Clear error flag */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_CMF);		//Character Match Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);		//IDLE line detected Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);		//Overrun Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);		//Framing Error Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);		//Parity Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);		//Noise detected Clear Flag
		
		
		//__HAL_UNLOCK(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, usart2_rx_dma_buf, sizeof(usart2_rx_dma_buf));
	}
	else
	{
		#ifdef __ENABLE_SHELL
		printf("Other USART2 Error!\r\n");
		#endif
	}
}

/**
  * @brief  UART2 Init
  * @param  None
  * @retval None
  *			Uart should config to DMA Rx circular mode and enable interrupt
  */
void USART2_Init(void)
{
	/* Init LwRB ring fifo */
	lwrb_init(&usart2_rx_rb, usart2_rx_rb_data, sizeof(usart2_rx_rb_data));
	
	/* Register rx event call back */
	HAL_UART_RegisterRxEventCallback(&huart2, USART2_RxEventCb);
	
	/* Register error call back */
	HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, USART2_ErrorCb);
	
	/* Start UART */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_dma_buf, sizeof(usart2_rx_dma_buf));
	
	/* Disable error interrupt to prevent unexpected crashes*/
	#if 1
	ATOMIC_CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE);
	//ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
	#endif
}

/**
  * @brief  Block mode receive N byte until timeout
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
HAL_StatusTypeDef USART2_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint32_t tick_start;
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
	
    /* Init tickstart for timeout management */
    tick_start = HAL_GetTick();
	while (lwrb_get_full(&usart2_rx_rb) < Size)
	{
		if (((HAL_GetTick() - tick_start) > Timeout) || (Timeout == 0U))
		{
			return HAL_TIMEOUT;
		}
	}
	
	lwrb_read(&usart2_rx_rb, pData, Size);
	return HAL_OK;
}

/**
  * @brief  Receive data from Ring Buffer
  * @param  Received data
  * @param	Max data size
  * @retval Actual received data length
  */
uint16_t USART2_ReadRB(uint8_t *pData, uint16_t MaxSize)
{
    if ((pData == NULL) || (MaxSize == 0U))
    {
      return  0u;
    }
	uint16_t RecvSize = lwrb_get_full(&usart2_rx_rb);
	uint16_t Size = RecvSize < MaxSize ? RecvSize : MaxSize;
	lwrb_read(&usart2_rx_rb, pData, Size);
	return Size;
}

/**
  * @brief  Clear ring buffer data
  * @param  None
  * @retval None
  */
void USART2_Reset()
{
	/* Reset ring buffer */
	lwrb_reset(&usart2_rx_rb);
}


HAL_StatusTypeDef USART2_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	return HAL_UART_Transmit(&huart2, pData, Size, Timeout);
}

#endif

/* USART3 --------------------------------------------------------------------*/
#ifdef USE_USART3

#define UART3_RX_DMA_BUF_LEN	(32u)
#define UART3_RX_RB_LEN			(129u)


extern osSemaphoreId_t	Usart3RxSemHandle;

/* DMA buffer */
uint8_t	usart3_rx_dma_buf[UART3_RX_DMA_BUF_LEN] __attribute__((aligned(32)));

/* LwRB instance and buffer */
lwrb_t	usart3_rx_rb;							// Ring buffer instance for RX data
uint8_t	usart3_rx_rb_data[UART3_RX_RB_LEN];		// Ring buffer data array for RX DMA


/**
  * @brief  Usr defined Rx Event Callback
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
void USART3_RxEventCb(UART_HandleTypeDef *huart, uint16_t size)
{
	UNUSED(size);	
	static uint16_t	pos_last;
	uint16_t		pos;
	
	#ifdef CACHE_SUPPORT
	/* Invalidate DCache for CM7 core */
	SCB_InvalidateDCache_by_Addr((uint32_t *)&usart3_rx_dma_buf, sizeof(usart3_rx_dma_buf));
	#endif
	
	pos = sizeof(usart3_rx_dma_buf) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (pos != pos_last)
	{
		if (pos > pos_last)
		{	/* Current position is over previous one */
			/*
			 * Processing is done in "linear" mode.
			 *
			 * Application processing is fast with single data block,
			 * length is simply calculated by subtracting pointers
			 *
			 * [   0   ]
			 * [   1   ] <- old_pos |------------------------------------|
			 * [   2   ]            |                                    |
			 * [   3   ]            | Single block (len = pos - old_pos) |
			 * [   4   ]            |                                    |
			 * [   5   ]            |------------------------------------|
			 * [   6   ] <- pos
			 * [   7   ]
			 * [ N - 1 ]
			 */
			if (lwrb_write(&usart3_rx_rb, &usart3_rx_dma_buf[pos_last], pos - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart3 lwrb write fail!\r\n");
				#endif
			}
		}
		else if (pos < pos_last)
		{
			/* "overflow" mode..
			 *
			 * Application must process data twice,
			 * since there are 2 linear memory blocks to handle
			 *
			 * [   0   ]            |---------------------------------|
			 * [   1   ]            | Second block (len = pos)        |
			 * [   2   ]            |---------------------------------|
			 * [   3   ] <- pos
			 * [   4   ] <- old_pos |---------------------------------|
			 * [   5   ]            |                                 |
			 * [   6   ]            | First block (len = N - old_pos) |
			 * [   7   ]            |                                 |
			 * [ N - 1 ]            |---------------------------------|
			 */
			if (lwrb_write(&usart3_rx_rb, &usart3_rx_dma_buf[pos_last], sizeof(usart3_rx_dma_buf) - pos_last) == NULL)
			{
				#ifdef __ENABLE_SHELL
				printf("usart3 lwrb write fail!\r\n");
				#endif
			}
			
			if (pos > 0)	/* Second block process */
			{
				if (lwrb_write(&usart3_rx_rb, &usart3_rx_dma_buf[0], pos) == NULL)
				{
					#ifdef __ENABLE_SHELL
					printf("usart3 lwrb write fail!\r\n");
					#endif
				}
			}
		}
		pos_last = pos;		/* Save current position as old for next transfers */
	}	/* if (pos != pos_last) */
	
	switch (huart->RxEventType)
	{
		case HAL_UART_RXEVENT_IDLE:
			osSemaphoreRelease(Usart3RxSemHandle);
			break;		
		
		case HAL_UART_RXEVENT_TC: 
			break;
		
		case HAL_UART_RXEVENT_HT:
			break;
		
		default:
			#ifdef __ENABLE_SHELL
			printf("RxEventType Error!\r\n");
			#endif
			break;
	}
}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void USART3_ErrorCb(UART_HandleTypeDef *huart)
{
	#ifdef __ENABLE_SHELL
	printf("USART3 Error!\r\n");
	#endif
	
	if (huart == &huart3)
	{
		/* Uart2 connected to master serial port */

		/* Clear error flag */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_CMF);		//Character Match Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);		//IDLE line detected Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);		//Overrun Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);		//Framing Error Clear Flag
		
		/* Not sure if it's useful */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);		//Parity Error Clear Flag
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);		//Noise detected Clear Flag
		
		
		//__HAL_UNLOCK(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, usart3_rx_dma_buf, sizeof(usart3_rx_dma_buf));
	}
	else
	{
		#ifdef __ENABLE_SHELL
		printf("Other USART3 Error!\r\n");
		#endif
	}
}

/**
  * @brief  UART3 Init
  * @param  None
  * @retval None
  *			Uart should config to DMA Rx circular mode and enable interrupt
  */
void USART3_Init(void)
{
	/* Init LwRB ring fifo */
	lwrb_init(&usart3_rx_rb, usart3_rx_rb_data, sizeof(usart3_rx_rb_data));
	
	/* Register rx event call back */
	HAL_UART_RegisterRxEventCallback(&huart3, USART3_RxEventCb);
	
	/* Register error call back */
	HAL_UART_RegisterCallback(&huart3, HAL_UART_ERROR_CB_ID, USART3_ErrorCb);
	
	/* Start UART */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_dma_buf, sizeof(usart3_rx_dma_buf));
	
	/* Disable error interrupt to prevent unexpected crashes*/
	#if 1
	ATOMIC_CLEAR_BIT(huart3.Instance->CR3, USART_CR3_EIE);
	//ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
	#endif
}

/**
  * @brief  Block mode receive N byte until timeout
  * @param  huart UART handle.
  * @param	received data size
  * @retval None
  */
HAL_StatusTypeDef USART3_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint32_t tick_start;
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
	
    /* Init tickstart for timeout management */
    tick_start = HAL_GetTick();
	while (lwrb_get_full(&usart3_rx_rb) < Size)
	{
		if (((HAL_GetTick() - tick_start) > Timeout) || (Timeout == 0U))
		{
			return HAL_TIMEOUT;
		}
	}
	
	lwrb_read(&usart3_rx_rb, pData, Size);
	return HAL_OK;
}

/**
  * @brief  Receive data from Ring Buffer
  * @param  Received data
  * @param	Max data size
  * @retval Actual received data length
  */
uint16_t USART3_ReadRB(uint8_t *pData, uint16_t MaxSize)
{
    if ((pData == NULL) || (MaxSize == 0U))
    {
      return  0u;
    }
	uint16_t RecvSize = lwrb_get_full(&usart3_rx_rb);
	uint16_t Size = RecvSize < MaxSize ? RecvSize : MaxSize;
	lwrb_read(&usart3_rx_rb, pData, Size);
	return Size;
}

/**
  * @brief  Clear ring buffer data
  * @param  None
  * @retval None
  */
void USART3_Reset()
{
	/* Reset ring buffer */
	lwrb_reset(&usart3_rx_rb);
}


HAL_StatusTypeDef USART3_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	return HAL_UART_Transmit(&huart3, pData, Size, Timeout);
}

#endif




///**
//  * @brief  Repeat mode start.
//  * @param  None
//  * @retval None
//  */
//void UART_IT_Start()
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
//	
//	ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
//	ATOMIC_CLEAR_BIT(huart3.Instance->CR3, USART_CR3_EIE);
//}


/* Redefine fputc() to use UART1 for printf() function */
/* ---------------------------------------------------------------------------*/
#if 0
/* UART1 */
#define AC6_ENABLE

#ifdef AC6_ENABLE
__ASM(".global __use_no_semihosting");
#else
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

/* Define _sys_exit() to avoid using half-host mode */
void _sys_exit(int x)
{
    x = x;
}
/* Redefine fputc */
int fputc(int ch, FILE *f)
{
    //ITM_SendChar(ch);
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
    ch = ch;
}
#endif

