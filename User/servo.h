#include "stm32f4xx.h"
#ifndef __servo_h
#define __servo_h

// 舵机逻辑ID（与pwm.h中的物理通道号对应）
// 注意：不要在此文件中重新定义SERVO_WAIST等宏
// 统一使用pwm.h中的定义
void Servo_SetAngle(uint8_t id, uint16_t angle);
void Servo_Init(void);

#endif
