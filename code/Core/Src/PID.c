#include "PID.h"

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max_output, float min_output, float integral_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->integral_limit = integral_limit;

    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    pid->output = 0.0f;
    pid->last_error = 0.0f;
    pid->current_error = 0.0f;
    pid->sum_error = 0.0f;
}

float PID_calculate(PID *pid, float current_value, float target_value)
{
    pid->current_error = target_value - current_value;
    float integral = pid->sum_error + pid->current_error;

    // 积分限幅
    if (integral > pid->integral_limit)
    {
        integral = pid->integral_limit;
    }
    if (integral < -pid->integral_limit)
    {
        integral = -pid->integral_limit;
    }

    pid->output = (pid->Kp) * (pid->current_error) + (pid->Ki) * integral + (pid->Kd) * ((pid->current_error) - (pid->last_error));

    // 输出限幅
    pid->output = (pid->output) > (pid->max_output) ? (pid->max_output) : (pid->output);
    pid->output = (pid->output) < (pid->min_output) ? (pid->min_output) : (pid->output);
    pid->last_error = pid->current_error;

    return pid->output;
}
