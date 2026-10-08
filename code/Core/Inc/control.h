#ifndef __CONTROL_H
#define __CONTROL_H

#include "PID.h"
#include "FOC.h"
#include "AS5600.h"

/* 调试用，可直接在调试器里观察 */
extern float current_angle; /* 机械角 0~360 */
extern float elec_angle;    /* 电角度 0~360 */
extern float pid_error;     /* 归一化后的角度误差 */
extern float pid_output;    /* PID 输出（float） */
extern int16_t uq_out;      /* 送给 FOC 的 Uq */

void position_control_init(void); /* 上电调用一次 */
void position_control(float target_angle);

#endif
