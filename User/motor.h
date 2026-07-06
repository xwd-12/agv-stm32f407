#include "stm32f4xx.h"
#ifndef __motor_h
#define __motor_h
#define MOTOR_LEFT_FRONT   0   // ��ǰ��
#define MOTOR_RIGHT_FRONT  1   // ��ǰ��
#define MOTOR_LEFT_REAR    2   // �����
#define MOTOR_RIGHT_REAR   3   // �Һ���
void Motor_EmergencyBrake(void);
void Motor_Init(void);
void Motor_SetSpeed(uint8_t id, int16_t speed);


#endif
