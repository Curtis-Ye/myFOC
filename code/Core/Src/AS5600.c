#include "AS5600.h"

extern I2C_HandleTypeDef hi2c1;

/* ==================== 调试用状态 ==================== */

/* HAL 返回的状态码 */
volatile HAL_StatusTypeDef AS5600_LastStatus = HAL_OK;
/* hi2c1.ErrorCode —— 注意：阻塞式 API 永远不会在这里出现 BERR/ARLO/OVR！ */
volatile uint32_t AS5600_LastError = 0;
/* ★ 原始寄存器快照。HAL 的轮询路径根本不检查 BERR/ARLO/OVR，
   所以想知道总线到底出了什么事，必须直接看 SR1/SR2。
   注意 SR1/SR2 都是 16 位寄存器，BERR/ARLO/AF/OVR 在 bit8~bit11，
   用 uint8_t 存会把它们全截掉！
     SR1: bit8=BERR 总线错误  bit9=ARLO 仲裁丢失  bit10=AF 无应答  bit11=OVR 溢出
     SR2: bit1=BUSY  总线忙  */
volatile uint16_t AS5600_LastSR1 = 0;
volatile uint16_t AS5600_LastSR2 = 0;

/* 统计量，用来判断是偶发还是持续性故障 */
volatile uint32_t AS5600_FailCount = 0;    /* 累计失败次数 */
volatile uint32_t AS5600_ConsecFail = 0;   /* 连续失败次数（成功即清零） */
volatile uint32_t AS5600_RecoverCount = 0; /* 触发总线恢复的次数 */

/* ==================== 磁场诊断 ==================== */
volatile uint8_t AS5600_Status = 0;
volatile uint8_t AS5600_Agc = 0;
volatile uint16_t AS5600_Magnitude = 0;

/* ==================== 参数 ==================== */
volatile static uint64_t total_machine_angle = UINT64_MAX / 2;
/* 单次传输超时。原来用 100ms —— 一次总线卡死就要瞎掉 100ms（=200 个控制周期）。
   注意：开头等待 BUSY 标志清 0 的那一步用的是 HAL 内部的固定 25ms，不受这个参数影响。 */
#define AS5600_TIMEOUT_MS 3U

/* 连续失败多少次后做完整恢复 */
#define AS5600_MAX_CONSEC_FAIL 3U

/* ~5µs 忙等 @72MHz */
static void i2c_short_delay(void)
{
    for (volatile uint32_t i = 0; i < 90U; i++)
    {
    }
}

/* 总线解锁：从机如果在传输中途被复位（或上电时序不对），可能把 SDA 拉死不放。
   这时主机连 START 都发不出来，BUSY 标志会一直卡住。
   办法是用 GPIO 手动打 9 个 SCL 脉冲，把从机的移位寄存器走完，逼它释放 SDA。
   ★ 必须在 HAL_I2C_DeInit() 之后调用，否则 PB6/PB7 还握在 I2C 外设手里。 */
static void i2c_bus_unlock(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* 释放 SDA */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); /* SCL = 1 */
    i2c_short_delay();

    /* 打 9 个时钟，把从机剩余的状态位全部移出去 */
    for (uint8_t i = 0; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        i2c_short_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        i2c_short_delay();
    }

    /* 补一个 STOP 条件：SCL 保持高，SDA 由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    i2c_short_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    i2c_short_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    i2c_short_delay();
}

/* 完整恢复：先复位外设拿回引脚，再解锁总线，最后重新初始化外设。
   这一步能同时解决三种卡死：
     1) hi2c->State 卡在 BUSY_RX（后续调用直接返回 HAL_BUSY）
     2) BUSY 标志卡住（START 发不出去）
     3) 从机把 SDA 拉死 */
void AS5600_I2C_Recover(void)
{
    HAL_I2C_DeInit(&hi2c1); /* 关外设时钟 + 释放 PB6/PB7 回 GPIO */
    i2c_bus_unlock();
    MX_I2C1_Init(); /* 重新初始化，引脚交还 I2C */
    AS5600_RecoverCount++;
}

/* ==================== 主接口 ==================== */

uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t data[2];
    HAL_StatusTypeDef status = HAL_ERROR;

    /* 失败重试一次。偶发错误（总线毛刺）重试通常就能过 */
    for (uint8_t retry = 0; retry < 2U; retry++)
    {
        status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, AS5600_REG_ANGLE_H, I2C_MEMADD_SIZE_8BIT, data, 2, AS5600_TIMEOUT_MS);

        if (status == HAL_OK)
        {
            AS5600_LastStatus = status;
            AS5600_LastError = hi2c1.ErrorCode;
            AS5600_ConsecFail = 0;
            /* 成功时结果被 & 0x0FFF，永远不可能是 0xFFFF */
            return (uint16_t)((((uint16_t)data[0] << 8) | data[1]) & 0x0FFF);
        }

        /* 失败：抓原始寄存器。ErrorCode 看不到 BERR/ARLO/OVR，必须直接读 SR1 */
        AS5600_LastStatus = status;
        AS5600_LastError = hi2c1.ErrorCode;
        AS5600_LastSR1 = hi2c1.Instance->SR1;
        AS5600_LastSR2 = hi2c1.Instance->SR2;
    }

    AS5600_FailCount++;
    AS5600_ConsecFail++;

    /* 连续失败到阈值 —— 说明外设或总线已经卡死，做完整恢复 */
    if (AS5600_ConsecFail >= AS5600_MAX_CONSEC_FAIL)
    {
        AS5600_I2C_Recover();
    }

    return 0xFFFF;
}

float AS5600_ReadAngle(void)
{
    uint16_t raw_angle;

    raw_angle = AS5600_ReadRawAngle();

    /* 通信失败时返回负数，避免把 0xFFFF 换算成 5759.9° 这种无意义的角度 */
    if (raw_angle == 0xFFFF)
        return -1.0f;

    return (raw_angle * 360.0f) / 4096.0f;
}

/* 探测指定 7 位地址上是否有器件应答，用于确认 AS5600 到底挂在哪个地址 */
HAL_StatusTypeDef AS5600_Probe(uint8_t addr_7bit)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr_7bit << 1), 3, 100);
}

/* 读磁场诊断寄存器。角度范围不对时，这三个值能直接说明是磁场的问题还是机械的问题。
   期望值：Status == 0x20（MD=1, ML=0, MH=0）、Agc 落在量程中段、Magnitude 明显偏离 0 */
void AS5600_ReadDiagnostics(void)
{
    uint8_t b;
    uint8_t m[2];

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x0B, I2C_MEMADD_SIZE_8BIT, &b, 1, 100) == HAL_OK)
        AS5600_Status = b;

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &b, 1, 100) == HAL_OK)
        AS5600_Agc = b;

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, m, 2, 100) == HAL_OK)
        AS5600_Magnitude = (uint16_t)((((uint16_t)m[0] << 8) | m[1]) & 0x0FFF);
}

void encoder_zeroing(void)
{
    /* delay to wait for the power supply voltage to be normal */
    delayms(1000);

    /* set that there is only a magnetic field on the straight axis. */
    // foc_calculate_dutycycle(0, CALI_TORQUE, 0, &u, &v, &w);
    // update_pwm_dutycycle(u, v, w);
    FOC_control(0, open_loop_Uq);
    delayms(300);
    elec_angle_offset = 0;
    total_machine_angle = UINT64_MAX / 2;

    /* read the angle at this time as the offset angle */
    elec_angle_offset = AS5600_ReadAngle();

    /* zero the torque in all directions to release the motor */
    // foc_calculate_dutycycle(0, 0, 0, &u, &v, &w);
    // update_pwm_dutycycle(u, v, w);
    FOC_control(0, 0);
    delayms(300);
}
