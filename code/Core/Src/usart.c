/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdlib.h>

/* USER CODE BEGIN 0 */
#define RX_BUFFER_SIZE 32

char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_data;
volatile uint8_t rx_complete = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */

    /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */

    /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspDeInit 1 */

    /* USER CODE END USART2_MspDeInit 1 */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* ★ 关键修正：Windows/串口助手的行结束符是 CRLF（"\r\n"）两个字符。
       收到 '\r' 时已经把字符串收尾、rx_index 归零；
       紧接着 '\n' 又进来，若不再判断一次，就会执行 rx_buffer[0] = '\0'，
       把整行的第一个字符抹成 0 —— 这就是"第一个元素一直是 0"的原因。
       用 rx_index > 0 判断"这一行确实收到过内容"，重复的结束符直接忽略。 */
    if (rx_data == '\n' || rx_data == '\r')
    {
      if (rx_index > 0U)
      {
        rx_buffer[rx_index] = '\0'; // 字符串结束符
        rx_complete = 1;            // 标记接收完成
        rx_index = 0;               // 准备下一次接收
      }
      /* rx_index == 0 说明前一个字符已经是行结束符，这个 '\n' 忽略掉 */
    }
    else
    {
      if (rx_index < RX_BUFFER_SIZE - 1)
      {
        rx_buffer[rx_index] = rx_data;
        rx_index++;
      }
    }
    // 重新开启接收中断（用 huart，不要硬编码 &huart2）
    HAL_UART_Receive_IT(huart, &rx_data, 1);
  }
}

/* UART 出错（ORE 溢出 / FE 帧错误 / NE 噪声 / PE 校验错）时，
   HAL 会中止接收并置 RxState = READY，但【不会自动重启】。
   不加这个回调，串口就永久哑了 —— 而 115200 下偶发噪声很常见。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* F1 上这四个宏都映射到"读 SR 再读 DR"的清标志序列 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    rx_index = 0; /* 出错时这一行作废，等下一行重新同步 */
    HAL_UART_Receive_IT(huart, &rx_data, 1);
  }
}

float get_angle(void)
{
  float angle = atof(rx_buffer);
  return angle;
}
