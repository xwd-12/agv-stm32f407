#include "stm32f4xx.h"
#include "pwm.h"

// 紧急刹车：上电第一时间刹车，在OLED/UART慢速外设初始化之前执行
// 避免电机驱动IN引脚浮空导致上电瞬间电机乱转
void Motor_EmergencyBrake(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int i;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

    // PE3, PE4, PE5, PE6
    for (i = 3; i <= 6; i++) {
        GPIO_InitStruct.GPIO_Pin = (1 << i);
        GPIO_Init(GPIOE, &GPIO_InitStruct);
        GPIO_WriteBit(GPIOE, (1 << i), Bit_RESET);
    }

    // PC8, PC4
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_4;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);

    // PD3, PD4
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
}
//�������

// ���巽�����Ŷ�Ӧ�� GPIO �˿ں�����
static GPIO_TypeDef* const IN1_PORT[] = {GPIOE, GPIOE, GPIOC, GPIOD};
static const uint16_t IN1_PIN[] = {GPIO_Pin_3, GPIO_Pin_5, GPIO_Pin_8, GPIO_Pin_3};
static GPIO_TypeDef* const IN2_PORT[] = {GPIOE, GPIOE, GPIOC, GPIOD};
static const uint16_t IN2_PIN[] = {GPIO_Pin_4, GPIO_Pin_6, GPIO_Pin_4, GPIO_Pin_4};

void Motor_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	   int i;
    // ʹ�� GPIO ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // ��ʼ�����з�������
	  
    for ( i = 0; i < 4; i++) {
        GPIO_InitStruct.GPIO_Pin = IN1_PIN[i];
        GPIO_Init(IN1_PORT[i], &GPIO_InitStruct);
        GPIO_InitStruct.GPIO_Pin = IN2_PIN[i];
        GPIO_Init(IN2_PORT[i], &GPIO_InitStruct);
        // Ĭ��ɲ����IN1=0, IN2=0��
        GPIO_WriteBit(IN1_PORT[i], IN1_PIN[i], Bit_RESET);
        GPIO_WriteBit(IN2_PORT[i], IN2_PIN[i], Bit_RESET);
    }
}
//����ָ��������ٶȺͷ���
void Motor_SetSpeed(uint8_t id, int16_t speed)
{
	  uint32_t duty = 0;
    if (id > 3) return;
    // �޷�
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed > 0) {          // ��ת
        GPIO_WriteBit(IN1_PORT[id], IN1_PIN[id], Bit_SET);
        GPIO_WriteBit(IN2_PORT[id], IN2_PIN[id], Bit_RESET);
        duty = (uint32_t)speed * 999 / 1000;   // ӳ�䵽 0~49��speedӳ�䵽duty����ʾ
    } else if (speed < 0) {    // ��ת
        GPIO_WriteBit(IN1_PORT[id], IN1_PIN[id], Bit_RESET);
        GPIO_WriteBit(IN2_PORT[id], IN2_PIN[id], Bit_SET);
        duty = (uint32_t)(-speed) * 999 / 1000;
    } else {                   // ֹͣ
        GPIO_WriteBit(IN1_PORT[id], IN1_PIN[id], Bit_RESET);
        GPIO_WriteBit(IN2_PORT[id], IN2_PIN[id], Bit_RESET);
        duty = 0;
    }
    PWM_Setcompare(id, duty);//����dutyֵ�ı�ccr�Ӷ��ı�ռ�ձ�
}
