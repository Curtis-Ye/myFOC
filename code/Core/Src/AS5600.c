#include "AS5600.h"

extern I2C_HandleTypeDef hi2c1;

/* 调试用：出错时把状态码留下来，便于在调试器里区分原因
   HAL_ERROR      -> 通常是 AF（从机不应答），地址错 / 器件没接 / 没上拉
   HAL_TIMEOUT    -> 总线卡住，SDA 或 SCL 被拉死，多数是缺上拉电阻
   HAL_BUSY       -> 上一次传输还没结束
*/

volatile HAL_StatusTypeDef AS5600_LastStatus = HAL_OK;
volatile uint32_t AS5600_LastError = 0;

/* 磁场诊断，角度异常时先看这三个变量 */
volatile uint8_t AS5600_Status = 0;
volatile uint8_t AS5600_Agc = 0;
volatile uint16_t AS5600_Magnitude = 0;

uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t data[2];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, AS5600_REG_ANGLE_H, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

    AS5600_LastStatus = status;
    AS5600_LastError = hi2c1.ErrorCode;

    if (status != HAL_OK)
        return 0xFFFF; /* 注意：成功读取时结果被 & 0x0FFF，永远不可能是 0xFFFF，
                          所以 0xFFFF 一定代表"通信失败"，而不是"角度 = 4095" */

    uint16_t raw_angle;

    raw_angle = ((uint16_t)data[0] << 8) | data[1];
    raw_angle &= 0x0FFF;

    return raw_angle;
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
