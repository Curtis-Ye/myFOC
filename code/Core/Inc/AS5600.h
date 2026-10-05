#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include "i2c.h"

#define AS5600_I2C_ADDR (0x35 << 1)

#define AS5600_REG_ANGLE_H 0x0C
#define AS5600_REG_ANGLE_L 0x0D

uint16_t AS5600_ReadRawAngle(void);
float AS5600_ReadAngle(void);

#endif
