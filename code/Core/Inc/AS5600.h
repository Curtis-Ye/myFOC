#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include "i2c.h"
#include "stdint.h"
#include "FOC.h"
#include "control.h"

/* AS5600 的 7 位固定地址是 0x36。
   HAL_I2C_* 接口需要"已左移 1 位"的 8 位形式，所以 << 1 得到 0x6C。 */

#define AS5600_I2C_ADDR (0x36 << 1)

#define AS5600_REG_ANGLE_H 0x0C
#define AS5600_REG_ANGLE_L 0x0D

/* 调试用：最近一次的 HAL 状态码和 I2C 错误码，直接在调试器里观察 */
extern volatile HAL_StatusTypeDef AS5600_LastStatus;
extern volatile uint32_t AS5600_LastError;

/* ★ 原始寄存器快照。
   HAL 的阻塞式（轮询）API 完全不检查 BERR/ARLO/OVR —— 它们只在中断/DMA 路径里处理。
   所以 ErrorCode 永远看不到总线级错误，想定位必须直接看 SR1/SR2：
     SR1: bit8=BERR 总线错误  bit9=ARLO 仲裁丢失  bit10=AF 无应答  bit11=OVR 溢出
     SR2: bit1=BUSY 总线忙 */
#define AS5600_SR1_BERR 0x0100U
#define AS5600_SR1_ARLO 0x0200U
#define AS5600_SR1_AF 0x0400U
#define AS5600_SR1_OVR 0x0800U
#define AS5600_SR2_BUSY 0x0002U

extern volatile uint16_t AS5600_LastSR1; /* 16 位！错误位在 bit8~bit11 */
extern volatile uint16_t AS5600_LastSR2; /* bit1 是 BUSY */

/* 故障统计：判断是偶发毛刺还是持续性卡死 */
extern volatile uint32_t AS5600_FailCount;    /* 累计失败 */
extern volatile uint32_t AS5600_ConsecFail;   /* 连续失败（成功即清零） */
extern volatile uint32_t AS5600_RecoverCount; /* 触发总线恢复的次数 */

/* 诊断寄存器：用于判断磁场是否合格（角度异常时先看这三个） */
#define AS5600_STATUS_MD 0x20U /* 检测到磁铁 */
#define AS5600_STATUS_ML 0x10U /* 磁场过弱 */
#define AS5600_STATUS_MH 0x08U /* 磁场过强 */

extern volatile uint8_t AS5600_Status;     /* 0x0B: bit5=MD, bit4=ML, bit3=MH */
extern volatile uint8_t AS5600_Agc;        /* 0x1A: 自动增益，3.3V 供电时范围 0..128 */
extern volatile uint16_t AS5600_Magnitude; /* 0x1B/0x1C: 磁场幅值 0..4095 */

uint16_t AS5600_ReadRawAngle(void);
float AS5600_ReadAngle(void);
HAL_StatusTypeDef AS5600_Probe(uint8_t addr_7bit);
void AS5600_ReadDiagnostics(void);
void AS5600_I2C_Recover(void); /* 外设复位 + 总线解锁，手动调用也行 */
void encoder_zeroing(void);
#endif
