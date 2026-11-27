#include "control.h"

/* ============================ 可调参数 ============================ */

/* Uq 上限。满幅 32767 会让占空比顶到 99.9%，自举电容来不及充电，
   所以留余量把峰值占空比压到 96% 以下。 */
#define UQ_MAX 30000.0f

/* 速度环调试期限幅：先压到约 10% 满幅，避免反馈噪声把 Uq 顶到满幅翻转、
   让电机啸叫或驱动过流。稳定后再逐步放开；只作用于速度环，不影响位置环。 */
#define VEL_UQ_MAX 6000.0f

/* 速度函数调用频率，必须和实际调用位置一致（TIM3 中断或主循环节拍）。
   TIM3: 72MHz / (71+1)  = 1000 kHz，见 tim.c 的 MX_TIM3_Init()。
   PID 的 Kp/Ki/Kd 都以这个周期为前提，改了这里参数要重调。 */
#define CTRL_FREQ_HZ 200.0f

/* 分频累计测速：每 20 个控制周期（10 ms）用「窗口起点角度 -> 当前角度」算一次速度。
   12 位编码器 1 LSB = 0.088°，单周期测速的分辨率只有 0.088*2000 = 176 °/s，
   低速根本测不到；分频后分辨率 = 0.088 / 0.01 = 8.8 °/s。
   顺带把主循环 I2C 更新与中断不同步带来的节拍抖动平均掉了。 */
#define VEL_DIV 10U

/* 速度一阶低通系数。0.2 对应约 50 ms 滞后，对 100 Hz 速度环偏大、容易超调；
   提到 0.4 后时间常数约 25 ms，相位裕度明显改善。
   若静止时速度读数噪声变大，可在 0.3~0.5 之间折中。 */
#define VEL_LPF_ALPHA 0.5f

#define VEL_SAMPLE_DT 0.01f

/* ============================ 全局量 ============================ */
/* 都放在这里，方便调试器直接观察 */

PID pos_pid;
PID vel_pid;

volatile float g_elec_angle = 0.0f; /* 换算后的电角度，0 ~ 360 */
float pid_error = 0.0f;             /* 归一化后的角度误差，调试用 */
float pid_output = 0.0f;            /* PID 输出（float），调试用 */
volatile float delta_angle = 0.0f;  /* 窗口内累计角度差（度/VEL_DIV 周期），调试用 */
int16_t g_uq_out = 0;               /* 最终送给 FOC 的 Uq，调试用 */

/* Uq 输出方向符号：1 或 -1，调试器可实时改。
   当前力矩方向与编码器速度符号不匹配，速度环是正反馈：
   手拨一下电机就沿那个方向自持加速到满速，uq_out 恒为 ±3000（饱和）。
   翻转 Uq 等价于把电角度整体加 180°，能把正反馈改回负反馈。
   若改完变成「朝另一个方向飞转」，把它改回 1 并重新标定 elec_angle_offset
   （7 对极下 ±25.7° 机械角 = 180° 电角度）。 */
volatile int8_t uq_sign = 1;

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
    PID_Init(&pos_pid,
             /*Kp*/ 50.0f,
             /*Ki*/ 0.0f,
             /*Kd*/ 1000.0f,
             /*max_output*/ UQ_MAX,
             /*min_output*/ -UQ_MAX,
             /*integral_limit*/ 1000.0f);
}

void position_control(float target_angle)
{
    /* 位置环：误差先归一化，再交给 PID */
    pid_error = angle_error(target_angle, g_current_angle);
    pid_output = PID_calculate_error(&pos_pid, pid_error);

    /* 力矩方向与 Uq 符号的对应关系是硬件属性，位置环同样要乘 */
    g_uq_out = uq_from_float(pid_output * (float)uq_sign);
}

float get_velocity(void)
{
    static float vel_filt = 0.0f;
    static uint8_t vel_inited = 0;

    if (!vel_inited)
    {
        vel_inited = 1;
        g_last_angle = g_current_angle;
        delta_angle = 0.0f;
        return 0.0f;
    }

    float d = g_current_angle - g_last_angle;
    if (d > 180.0f)
        d -= 360.0f;
    else if (d <= -180.0f)
        d += 360.0f;

    g_last_angle = g_current_angle;
    delta_angle = d;

    float v_raw = d / VEL_SAMPLE_DT; /* 度 / 0.01s = 度/s */
    vel_filt += VEL_LPF_ALPHA * (v_raw - vel_filt);
    return vel_filt;
}

/* 速度环一般用PI控制，P用来控制响应速度，I用来消除稳态误差，
    D很少用，因为会放大编码器速度测量噪声。 */
void velocity_control_init(void)
{
    /* Kp=1000 时饱和阈值只有 30 °/s，必然形成极限环振荡，所以先降下来。
       当前为纯 P（Ki=0），用于先验证环路是否稳定；
       确认平稳后再按 0.2~0.5 的步长慢慢加 Ki 消稳态误差。
       PI 已改为按 100 Hz（测速窗口）计算，积分不再按 2 kHz 累加。 */
    PID_Init(&vel_pid,
             /*Kp*/ 5.0f,
             /*Ki*/ 0.1f,
             /*Kd*/ 0.0f,
             /*max_output*/ VEL_UQ_MAX,
             /*min_output*/ -VEL_UQ_MAX,
             /*integral_limit*/ 25000.0f);
}

void velocity_control(float target_velocity, float current_velocity)
{

    pid_error = target_velocity - current_velocity;
    pid_output = PID_calculate_error(&vel_pid, pid_error);

    /* pid_output 是 PID 原始输出，uq_out 是乘了方向符号后的实际输出，
   两者符号可能相反，看调试窗口时别混淆。 */
    g_uq_out = uq_from_float(pid_output * (float)uq_sign);
}
