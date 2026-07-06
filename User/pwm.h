#ifndef __pwm_h
#define __pwm_h
#include "stm32f4xx.h"
// ���ͨ��������
#define MOTOR_LEFT_FRONT   0   // ��ǰ��
#define MOTOR_RIGHT_FRONT  1   // ��ǰ��
#define MOTOR_LEFT_REAR    2   // �����
#define MOTOR_RIGHT_REAR   3   // �Һ���

// ���ͨ��������
#define SERVO_WAIST        4   // �������
#define SERVO_SHOULDER     5   // ��۶��
#define SERVO_ELBOW        6   // С�۶�� (已废弃, PA10复用作挂钩)
#define SERVO_HOOK         6   // 挂钩舵机 PA10 TIM1_CH3 (复用废弃小臂通道)
#define SERVO_GRIPPER      7   // ��צ���
#define SERVO_ELBOW_PA2    8   // 小臂舵机 PA2 (TIM9_CH1)
void Pwm_TIM_Init(void);
static void  	Servoshoulder_Init(void);//Tim1_1,2   pa8,9
static void  	ServoGripper_Init(void);//Tim1_3,4   pa10,11
void PWM_init(void);//������ʼ��
void Servo_PWMEnable(uint8_t id, uint8_t enable);
void PWM_Setcompare(uint8_t channel1, uint32_t pulse);//�����arr+1��50�������100



#endif
