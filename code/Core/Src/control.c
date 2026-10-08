#include "control.h"

/* ============================ 可调参数 ============================ */

/* Uq 上限。满幅 32767 会让占空比顶到 99.9%，自举电容来不及充电，
   所以留余量把峰值占空比压到 96% 以下。 */
#define UQ_MAX 30000.0f

/* 位置环调用频率，必须和实际调用位置一致（TIM3 中断或主循环节拍）。
   PID 的 Kp/Ki/Kd 都以这个周期为前提，改了这里参数要重调。 */
#define CTRL_FREQ_HZ 1000.0f

/* ============================ 全局量 ============================ */
/* 都放在这里，方便调试器直接观察 */

PID pid1;

float current_angle = 0.0f; /* AS5600 读到的机械角，0 ~ 360 */
float elec_angle = 0.0f;    /* 换算后的电角度，0 ~ 360 */
float pid_error = 0.0f;     /* 归一化后的角度误差，调试用 */
float pid_output = 0.0f;    /* PID 输出（float），调试用 */
int16_t uq_out = 0;         /* 最终送给 FOC 的 Uq，调试用 */

/* ============================ 内部函数 ============================ */

/* 角度差归一化到 (-180, 180]
   角度是圆周量：电机在 359°、目标 1° 时，真实误差是 +2° 而不是 -358°。
   不做这一步，电机会"绕远路"反向转 358°，或者来回震荡。 */
static float angle_error(float target, float current)
{
    float e = target - current;
    while (e > 180.0f)
        e -= 360.0f;
    while (e <= -180.0f)
        e += 360.0f;
    return e;
}

/* float -> int16_t 的正确姿势：先限幅，再四舍五入
   1) 必须先限幅：C 标准里 float 转整数时若值超出目标范围是"未定义行为"，
      不是"截断成边界值"。
   2) 四舍五入：直接 (int16_t) 是向零截断，会带来固定的 -0.5 LSB 系统偏差。 */
static int16_t uq_from_float(float uq)
{
    if (uq > UQ_MAX)
        uq = UQ_MAX;
    if (uq < -UQ_MAX)
        uq = -UQ_MAX;

    return (int16_t)((uq >= 0.0f) ? (uq + 0.5f) : (uq - 0.5f));
}

/* ============================ 对外接口 ============================ */

/* PID 只在启动时初始化一次。
   放在 position_control() 里会导致每个周期都清零 sum_error / last_error，
   积分和微分项永久失效。 */
void position_control_init(void)
{
    PID_Init(&pid1,
             /*Kp*/ 10.0f,
             /*Ki*/ 0.0f,
             /*Kd*/ 20.0f,
             /*max_output*/ UQ_MAX,
             /*min_output*/ -UQ_MAX,
             /*integral_limit*/ 1000.0f);
}

void position_control(float target_angle)
{

    /* 位置环：误差先归一化，再交给 PID */
    pid_error = angle_error(target_angle, current_angle);
    pid_output = PID_calculate_error(&pid1, pid_error);

    /* 机械角 -> 电角度。*/
    elec_angle = current_angle * (float)POLE_PAIRS;
    while (elec_angle >= 360.0f)
        elec_angle -= 360.0f;
    while (elec_angle < 0.0f)
        elec_angle += 360.0f;

    uq_out = uq_from_float(pid_output);
    FOC_control(elec_angle, uq_out);
}
