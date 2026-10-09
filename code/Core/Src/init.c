#include "init.h"

void peripheral_init(void)
{
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    position_control_init();
}
