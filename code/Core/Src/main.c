#include "init.h"
#include "control.h"

// #define OPEN_LOOP
// #define POSITION_LOOP

void SystemClock_Config(void);

/* 跨文件共享的变量统一在 control.h 声明，这里不再另写 extern */
volatile uint32_t g_angle_updates = 0; /* 编码器成功采样次数，用来看数据新鲜度 */
volatile uint32_t g_angle_errors = 0;  /* I2C 失败次数 */
extern uint8_t rx_data;
volatile float usart_angle = 0;    // 从串口获取的目标角度
volatile int32_t temp = 0;         // 速度观测值（LSB/s），调试用

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  peripheral_init(); // 外设初始化

  HAL_TIM_Base_Start_IT(&htim3);             // 启动定时器中断，用于开环控制
  HAL_UART_Receive_IT(&huart2, &rx_data, 1); // 启动串口接收中断

  /* TIM1 是高级控制定时器：必须启动 PWM 才会置位 MOE 并使能计数器，否则引脚恒为无效电平 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  while (1)
  {
    /* I2C 只在 TIM3 中断里采样，主循环不碰 hi2c1，避免阻塞式 HAL_I2C 被中断重入 */
    usart_angle = get_angle();
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    /* 编码器单点采样：整个工程只有这里访问 I2C，杜绝主循环 / 中断并发重入 */
    uint16_t raw = AS5600_ReadRawAngle();

    if (raw != 0xFFFF)
    {
      last_velocity_angle = current_velocity_angle;
      current_velocity_angle = raw;
      current_angle = raw * (360.0f / 4096.0f); /* 位置环与速度环共用同一份角度快照 */
      temp = get_raw_velocity();
      g_angle_updates++;
    }
    else
    {
      /* 本周期读数无效：保持上次角度、速度置 0，别让 0xFFFF 变成尖峰 */
      last_velocity_angle = current_velocity_angle;
      temp = 0;
      g_angle_errors++;
    }

#ifdef OPEN_LOOP
    FOC_OpenLoop_Update();
#endif

#ifdef POSITION_LOOP
    position_control(usart_angle);
#endif
  }
}
