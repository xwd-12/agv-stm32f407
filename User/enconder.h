#ifndef __enconder_h
#define __enconder_h
#include "stm32f4xx.h"

#define ENCODER_LEFT_FRONT   0
#define ENCODER_RIGHT_FRONT  1
#define ENCODER_LEFT_REAR    2
#define ENCODER_RIGHT_REAR   3

extern int16_t encoder_max[4];  // 每个编码器10ms内最大脉冲数，需根据电机实际最高转速分别标定

void Encoder_Init(void);
int16_t Encoder_getspeed(uint8_t idx);
int32_t Encoder_GetPosition(uint8_t idx);
void Encoder_ClearPosition(uint8_t idx);



#endif
