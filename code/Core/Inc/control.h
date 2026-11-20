#ifndef __CONTROL_H
#define __CONTROL_H

#include "PID.h"
#include "FOC.h"
#include "AS5600.h"

/* 调试用，可直接在调试器里观察
   注意：所有跨文件共享的变量只在这里声明一次，任何 .c 都不要另写 extern，
   否则类型写错时编译器和链接器都不会报错（float / uint16_t 踩过坑）。 */
extern volatile float current_angle; /* 机械角 0~360 */
extern float elec_angle;             /* 电角度 0~360 */
extern float pid_error;              /* 归一化后的角度误差 */
extern float pid_output;             /* PID 输出（float） */
extern int16_t uq_out;               /* 送给 FOC 的 Uq */

/* AS5600 原始角度快照，由 TIM3 中断单点采样，定义在 control.c */
extern volatile uint16_t current_velocity_angle; /* 本次中断读数 0~4095 */
extern volatile uint16_t last_velocity_angle;    /* 上一次中断读数 0~4095 */

void position_control_init(void); /* 上电调用一次 */
void position_control(float target_angle);
int32_t get_raw_velocity(void); /* 速度 LSB/s，必须用 32 位 */
#endif
