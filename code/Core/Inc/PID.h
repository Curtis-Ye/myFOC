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
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max_output, float min_output, float integral_limit);
void PID_Reset(PID *pid);
#endif
