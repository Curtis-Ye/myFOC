#include "stm32f1xx_hal.h"

/* 基于 HAL_GetTick() 的毫秒延时。
   原来用软件计数器忙等，但全工程没有任何地方递减 delayms_counter
   （SysTick_Handler 只调用了 HAL_IncTick），所以 delayms() 会永远卡在 while 里。
   HAL_GetTick() 由 HAL_IncTick() 在 1 kHz SysTick 中维护，直接用它最稳妥，
   无符号减法也能正确处理 uint32_t 回绕。 */
void delayms(unsigned long count)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < (uint32_t)count)
    {
    }
}
