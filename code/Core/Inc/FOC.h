#ifndef __FOC_H
#define __FOC_H
#include "stdint.h"

#define SIN_TAB_SIZE 1024
#define SIN_TAB_SHIFT 6        // 16 - log2(1024)
#define SIN_TAB_FRAC_MASK 0x3F // 63
#define SQRT3_2_Q15 28378      // sqrt(3)/2 * 32768

#define POLE_PAIRS 7           // 电机极对数
#define FOC_CALL_FREQ 10000.0f // 10kHz

typedef struct ElectricAngle
{
    int16_t U_alpha;
    int16_t U_beta;
} ElectricAngle;

typedef struct ThreePhase
{
    int16_t Ua;
    int16_t Ub;
    int16_t Uc;
} ThreePhase;

typedef struct Duty
{
    int32_t dutyA;
    int32_t dutyB;
    int32_t dutyC;
} Duty;

ElectricAngle park_inverse(uint16_t angle, int16_t Uq, int16_t Ud);
ThreePhase clark_inverse(int16_t U_alpha, int16_t U_beta);
void FOC_control(float current_angle, int16_t Uq);
void FOC_OpenLoop_Update(void);

#endif
