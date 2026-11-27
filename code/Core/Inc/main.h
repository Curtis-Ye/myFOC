/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#include "control.h"
#include "init.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

// #define OPEN_LOOP
 #define POSITION_LOOP
//#define VELOCITY_LOOP

  typedef enum Mode
  {
    SIMPLE,
    USART
  } Mode;

  extern uint8_t current_mode;
  extern volatile float usart_data;
  extern volatile float g_velocity;
  extern int16_t g_uq_out;
	extern float test_velocity;
  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
