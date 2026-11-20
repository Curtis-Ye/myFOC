#ifndef __CONTROL_H
#define __CONTROL_H

#include "PID.h"
#include "FOC.h"
#include "AS5600.h"

/* 调试用，可直接在调试器里观察
   注意：所有跨文件共享的变量只在这里声明一次，任何 .c 都不要另写 extern，
   否则类型写错时编译器和链接器都不会报错（float / uint16_t 踩过坑）。 */
extern volatile float current_angle; /* 当前机械角 0~360 */
extern volatile float last_angle;    /* 上一次机械角 0~360 */

extern float elec_angle; /* 电角度 0~360 */
extern float pid_error;  /* 归一化后的角度误差 */
extern float pid_output; /* PID 输出（float） */
extern int16_t uq_out;   /* 送给 FOC 的 Uq */
extern volatile float delta_angle; /* 角度差（度/采样周期），调试用 */

void position_control_init(void); /* 上电调用一次 */
void position_control(float target_angle);
float get_raw_velocity(void); /* 速度，单位 度/s；内部会更新 last_angle */
#endif
