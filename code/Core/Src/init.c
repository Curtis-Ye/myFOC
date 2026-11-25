#include "init.h"

void peripheral_init(void)
{
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();   /* TIM2 用于 PID 中断，必须在 HAL_TIM_Base_Start_IT(&htim2) 之前初始化 */
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    position_control_init();
	velocity_control_init();
}
