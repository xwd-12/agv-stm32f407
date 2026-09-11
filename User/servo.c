#include "stm32f4xx.h"
#include "pwm.h"
#include "servo.h"
#include "arm_config.h"
// ����ģ��Ķ�� ID ӳ�䵽 pwm.h �е�ͨ����
static const uint8_t servo_pwm_channel[] = {
    SERVO_WAIST,        // 4 �?PA8  (TIM1_CH1)
    SERVO_SHOULDER,     // 5 �?PA9  (TIM1_CH2)
    SERVO_ELBOW_PA2,    // 8 �?PA2 (TIM9_CH1)
    SERVO_GRIPPER,      // 7 �?PA11 (TIM1_CH4)
    SERVO_HOOK          // 6 �?PA10 (TIM1_CH3) 挂钩舵机
};

// 脉冲-角度映射
// 腰座(ID=0): 270°舵机, 500µs=0°, 2500µs=270°
// 其余(ID=1-4): 180°舵机, 500µs=0°, 2500µs=180°
#define PULSE_MIN    500
#define PULSE_MAX    2500
void Servo_SetAngle(uint8_t id, uint16_t angle)
{
	uint32_t pulse;
	uint16_t max_deg;
    if (id >= 5)
			return;

	/* 腰座270°, 其他180° */
	max_deg = (id == 0) ? 270 : 180;
    if (angle > max_deg)
			angle = max_deg;

    pulse = PULSE_MIN + (uint32_t)((angle * (PULSE_MAX - PULSE_MIN)) / max_deg);
    PWM_Setcompare(servo_pwm_channel[id], pulse);
}
//ȫ��������õ���ʼ�Ƕȣ�������?0��
void Servo_Init(void)
{
    Servo_SetAngle(0, ARM_HOME_WAIST);
    Servo_SetAngle(1, ARM_HOME_SHOULDER);
    Servo_SetAngle(2, ARM_HOME_ELBOW);
    Servo_SetAngle(3, ARM_HOME_GRIPPER);
    Servo_SetAngle(4, ARM_HOME_HOOK);  /* boot: hook LOCKED(10deg), trailer stays hooked */
}
