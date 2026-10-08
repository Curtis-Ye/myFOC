#ifndef __PID_H
#define __PID_H

typedef struct PID
{
    float Kp;
    float Ki;
    float Kd;

    float last_error;
    float current_error;
    float sum_error;

    float output;
    float max_output;
    float min_output;

    float integral_limit;

} PID;

float PID_calculate(PID *pid, float current_value, float target_value);
/* 直接以"误差"为输入，位置环用这个（需要调用方先把角度差归一化到 (-180,180]） */
float PID_calculate_error(PID *pid, float error);
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max_output, float min_output, float integral_limit);
void PID_Reset(PID *pid);
#endif
