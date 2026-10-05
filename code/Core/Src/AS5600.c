#include "AS5600.h"

extern I2C_HandleTypeDef hi2c1;

uint16_t AS5600_ReadRawAngle()
{
    uint8_t data[2];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, AS5600_REG_ANGLE_H, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

    if (status != HAL_OK)
        return 0xFFFF;

    uint16_t raw_angle;

    raw_angle = ((uint16_t)data[0] << 8) | data[1];
    raw_angle &= 0x0FFF;

    return raw_angle;
}

float AS5600_ReadAngle()
{
    uint16_t raw_angle;

    raw_angle = AS5600_ReadRawAngle();

    return (raw_angle * 360.0f) / 4096.0f;
}
