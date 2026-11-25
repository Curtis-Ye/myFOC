#include "stm32f1xx_hal.h"

static volatile unsigned long delayms_counter = 1;

void delayms(unsigned long count) {
    delayms_counter = count;
    while (0U != delayms_counter);
}
