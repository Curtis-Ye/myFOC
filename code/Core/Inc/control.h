#ifndef __CONTROL_H
#define __CONTROL_H

#include "PID.h"
#include "FOC.h"
#include "AS5600.h"

/* 调试用，可直接在调试器里观察
   注意：所有跨文件共享的变量只在这里声明一次，任何 .c 都不要另写 extern，
   否则类型写错时编译器和链接器都不会报错（float / uint16_t 踩过坑）。 */
extern volatile float g_elec_angle;       /* 电角度 0~360 */
extern float pid_error;                   /* 归一化后的角度误差 */
extern float pid_output;                  /* PID 输出（float） */
extern int16_t g_uq_out;                  /* 送给 FOC 的 Uq */
extern volatile float delta_angle;        /* 累计角度差（度/VEL_DIV 周期），调试用 */
extern volatile uint8_t velocity_updated; /* 新速度反馈待取走：get_velocity 置 1，velocity_control 清零 */
extern volatile int8_t uq_sign;           /* Uq 方向符号：1 / -1，调试器可实时改 */

void position_control_init(void); /* 上电调用一次 */
void position_control(float target_angle);
float get_velocity(void); /* 速度 度/s；每 VEL_DIV 周期更新一次，返回低通后的值 */
void velocity_control_init(void);
void velocity_control(float target_velocity, float current_velocity); /* 速度反馈由调用方每周期测一次后传入 */
#endif
