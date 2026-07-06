#include "stm32f4xx.h"
#include "pwm.h"
#include "servo.h"
#include "arm_config.h"
// ����ģ��Ķ�� ID ӳ�䵽 pwm.h �е�ͨ����
static const uint8_t servo_pwm_channel[] = {
    SERVO_WAIST,        // 4 → PA8  (TIM1_CH1)
    SERVO_SHOULDER,     // 5 → PA9  (TIM1_CH2)
    SERVO_ELBOW_PA2,    // 8 → PA2 (TIM9_CH1)
    SERVO_GRIPPER,      // 7 → PA11 (TIM1_CH4)
    SERVO_HOOK          // 6 → PA10 (TIM1_CH3) 挂钩舵机
};

// �Ƕ�ӳ���������Ҫ����ʵ�ʶ��У׼��
// 0�� ��Ӧ�� PWM �Ƚ�ֵ��ռ�ձȣ�
#define PULSE_0     500
// 180�� ��Ӧ�� PWM �Ƚ�ֵ
#define PULSE_180   2500
void Servo_SetAngle(uint8_t id, uint16_t angle)
{
	uint32_t pulse;
    if (id >= 5) 
			return;
    if (angle > 180) 
			angle = 180;

    // ���Բ�ֵ���� pulse
     pulse = PULSE_0 + (uint32_t)((angle * (PULSE_180 - PULSE_0)) / 180);
    PWM_Setcompare(servo_pwm_channel[id], pulse);
}
//ȫ��������õ���ʼ�Ƕȣ�������90��
void Servo_Init(void)
{
    Servo_SetAngle(0, ARM_HOME_WAIST);
    Servo_SetAngle(1, ARM_HOME_SHOULDER);
    Servo_SetAngle(2, ARM_HOME_ELBOW);
    Servo_SetAngle(3, ARM_HOME_GRIPPER);
    Servo_SetAngle(4, ARM_HOME_HOOK);
}
