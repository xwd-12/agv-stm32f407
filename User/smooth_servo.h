#include "stm32f4xx.h"
#include "stdbool.h"
#ifndef smooth_servo_h
#define smooth_servo_h
void smooth_Init(void);//��ʼ���ζ����pid�ṹ�������ֵ
int16_t smooth_Settarget(uint8_t id,uint16_t target_deg,uint16_t time_ms);
 void smoothservo_TimerHandle(void);//ÿ10ms����һ��
 bool smoothservo_IsBusy(uint8_t id);



void smoothservo_EmergencyStop(void);
void smoothservo_StopOne(uint8_t id);
bool smoothservo_AnyBusy(void);
uint8_t smoothservo_GetState(uint8_t id);
float smooth_GetCurrentAngle(uint8_t id);

#endif
