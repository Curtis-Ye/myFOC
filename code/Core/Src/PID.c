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

/* 核心：以"误差"为输入。
   位置环必须先把角度差归一化到 (-180, 180] 再送进来，
   否则 359° -> 1° 会被算成 -358° 的巨大误差。 */
float PID_calculate_error(PID *pid, float error)
{
    pid->current_error = error;

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

    // 关键：把"限幅后"的积分量存回去。
    // 原来漏了这一行，sum_error 永远是 0，积分项退化成 Ki*current_error（和 P 项重复），
    // 稳态误差永远消不掉。存限幅后的值即天然实现抗积分饱和（anti-windup）。
    pid->sum_error = integral;

    pid->last_error = pid->current_error;

    return pid->output;
}

/* 以"当前值/目标值"为输入的版本，误差 = target - current。
   注意：这是普通线性误差，不适合处理 0/360 环绕的角度量。 */
float PID_calculate(PID *pid, float current_value, float target_value)
{
    return PID_calculate_error(pid, target_value - current_value);
}
