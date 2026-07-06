#include "stm32f4xx.h"
#include "pwm.h"
#include "arm_config.h"

/* 计算舵机脉冲: angle 0-180 → 500-2500us */
#define SERVO_PULSE(a)  (uint32_t)(500 + ((a) * 2000UL) / 180)
//���pwm���
//�ĸ����ӵ����Ӧ���ĸ�pwmͨ��
void  Pwm_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_structure;
	TIM_OCInitTypeDef OC_Structure;
	GPIO_InitTypeDef GPIOinitstrcture = {0};

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_AF;      // ����ģʽ,��������afconfig��ʼ��
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_6; // pa6��ǰ��
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_NOPULL;       // ���죬����ʱ����Ϊ����
    GPIOinitstrcture.GPIO_Speed = 	GPIO_Speed_100MHz;  // 100hz
    GPIOinitstrcture.GPIO_OType = GPIO_OType_PP; 	//�������������Ȩ�ڶ�ʱ���Ǳߣ�����ȽϿ��ƣ�

	  GPIO_Init(GPIOA,&GPIOinitstrcture);//init��ʼ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);//������������

	  GPIOinitstrcture.GPIO_Pin = GPIO_Pin_7;//PA7 = TIM3_CH2, MOTOR0 PWM
		GPIO_Init(GPIOA,&GPIOinitstrcture);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);

	 GPIOinitstrcture.GPIO_Pin = GPIO_Pin_1;//PB1 = TIM3_CH4, 左后轮PWM
		GPIO_Init(GPIOB,&GPIOinitstrcture);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);

		GPIOinitstrcture.GPIO_Pin = GPIO_Pin_15;//pb15�Һ���
		GPIO_Init(GPIOB,&GPIOinitstrcture);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);

	TIM_InternalClockConfig(TIM3);//��ʱ���ڲ�ʱ�����ã���ʵĬ�Ͼ����ڲ�ʱ�ӣ�
	TIM_InternalClockConfig(TIM12);
	//APB1ʱ��Ϊ42hz������tim��ʱ��Ƶ��*2
	TIM_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_structure.TIM_CounterMode =TIM_CounterMode_Up ;//���ϼ���ģʽ
	TIM_structure.TIM_Period = 1000-1;//��װֵ1000, 分辨率0~999
	TIM_structure.TIM_Prescaler =84-1 ;//psc, PWM频率=84MHz/(84*1000)=1kHz
	TIM_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TIM_structure);

	TIM_OCStructInit(&OC_Structure);//�ṹ�����г�Ա��ʼ�����ã����ⲻ��Ҫ���õĳ�Ա��bug
	OC_Structure.TIM_OCMode=TIM_OCMode_PWM1;//���ģʽѡ�� ���� �м� pwm1 pwm2
	OC_Structure.TIM_OCPolarity=TIM_OCPolarity_High;//��ƽ���Է�ת����
	OC_Structure.TIM_OutputState=ENABLE;
	OC_Structure.TIM_Pulse = 0;//����ccr

	TIM_OC1Init(TIM3,&OC_Structure);
	TIM_OC1PolarityConfig(TIM3,TIM_OCPolarity_High);//�����ccrֵ֮ǰ���Ǹߵ�ƽ(��Ч)
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);   // Ӱ�ӼĴ������أ��޸ĵ�ccrֵ�ȴ������¼���������µ�һ�������
	TIM_OC2Init(TIM3,&OC_Structure);
	TIM_OC2PolarityConfig(TIM3,TIM_OCPolarity_High);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);   // ����һ��һ��TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3,&OC_Structure);
    TIM_OC4PolarityConfig(TIM3,TIM_OCPolarity_High);
    TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3,TIM_OCPreload_Enable);//����arr��Ӱ�ӼĴ���
	TIM_Cmd(TIM3,ENABLE);

	TIM_TimeBaseInit(TIM12,&TIM_structure);
	TIM_OC1Init(TIM12,&OC_Structure);
	TIM_OC1PolarityConfig(TIM12,TIM_OCPolarity_High);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM12,&OC_Structure);
	TIM_OC2PolarityConfig(TIM12,TIM_OCPolarity_High);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM12,TIM_OCPreload_Enable);//����arr��Ӱ�ӼĴ���
	TIM_Cmd(TIM12,ENABLE);

}

//��ʼ�����̺ʹ�۶��
static void  	Servoshoulder_Init(void)//Tim1_1,2   pa8,9
{	GPIO_InitTypeDef GPIOinitstrcture = {0};
	TIM_TimeBaseInitTypeDef TIM_structure;
	TIM_OCInitTypeDef OC_Structure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

	  GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_AF;      // ����ģʽ,��������afconfig��ʼ��
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_NOPULL;       // ���죬����ʱ����Ϊ����
    GPIOinitstrcture.GPIO_Speed = 	GPIO_Speed_50MHz;  // 50hz
    GPIOinitstrcture.GPIO_OType = GPIO_OType_PP;
	  GPIO_Init(GPIOA,&GPIOinitstrcture);//init��ʼ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
				GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
				GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);
	TIM_InternalClockConfig(TIM1);

	TIM_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_structure.TIM_CounterMode =TIM_CounterMode_Up ;//���ϼ���ģʽ
		TIM_structure.TIM_Period =20000-1;//ARR
		TIM_structure.TIM_Prescaler =168-1 ;//PSC
	TIM_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_structure);

	TIM_OCStructInit(&OC_Structure);//�ṹ�����г�Ա��ʼ�����ã����ⲻ��Ҫ���õĳ�Ա��bug
	OC_Structure.TIM_OCMode=TIM_OCMode_PWM1;//���ģʽѡ�� ���� �м� pwm1 pwm2
	OC_Structure.TIM_OCPolarity=TIM_OCPolarity_High;//��ƽ���Է�ת����
	OC_Structure.TIM_OutputState=ENABLE;
	OC_Structure.TIM_Pulse = 0;//����ccr

	TIM_OC1Init(TIM1,&OC_Structure);
	TIM_OC1PolarityConfig(TIM1,TIM_OCPolarity_High);

  TIM_OC2Init(TIM1,&OC_Structure);
	TIM_OC2PolarityConfig(TIM1,TIM_OCPolarity_High);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 先设置安全默认脉宽，再使能定时器，防止上电瞬间舵机乱动
    /* CH3(PA10)已废弃 — 小臂已改到PA2(TIM9_CH1), 禁用输出 */
    /* CH4(PA11) for gripper */
    TIM_OC3Init(TIM1, &OC_Structure);
    TIM_OC3PolarityConfig(TIM1, TIM_OCPolarity_High);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM1, &OC_Structure);
    TIM_OC4PolarityConfig(TIM1, TIM_OCPolarity_High);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_SetCompare4(TIM1, SERVO_PULSE(ARM_HOME_GRIPPER));

    TIM_SetCompare1(TIM1, SERVO_PULSE(ARM_HOME_WAIST));
    TIM_SetCompare2(TIM1, SERVO_PULSE(ARM_HOME_SHOULDER));

	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);

	/* PA10 复用为挂钩舵机 (原小臂已改到PA2/TIM9_CH1) */
	TIM_CCxCmd(TIM1, TIM_Channel_3, ENABLE);
}


//��ʼ��С�ۺͼ�צ��pwm
static void  	ServoGripper_Init(void)//Tim1_3,4   pa10,11
{
	GPIO_InitTypeDef GPIOinitstrcture = {0};
	TIM_TimeBaseInitTypeDef TIM_structure;
	TIM_OCInitTypeDef OC_Structure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

	  GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_AF;      // ����ģʽ,��������afconfig��ʼ��
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_10| GPIO_Pin_11; // pa10С�� pa11��צ
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_NOPULL;       // ���죬����ʱ����Ϊ����
    GPIOinitstrcture.GPIO_Speed = 	GPIO_Speed_50MHz;  // 50hz
    GPIOinitstrcture.GPIO_OType = GPIO_OType_PP;
	  GPIO_Init(GPIOA,&GPIOinitstrcture);//init��ʼ��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);
	TIM_InternalClockConfig(TIM1);

	TIM_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_structure.TIM_CounterMode =TIM_CounterMode_Up ;//���ϼ���ģʽ
		TIM_structure.TIM_Period =20000-1;//ARR
		TIM_structure.TIM_Prescaler =168-1 ;//PSC
	TIM_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_structure);

	TIM_OCStructInit(&OC_Structure);//�ṹ�����г�Ա��ʼ�����ã����ⲻ��Ҫ���õĳ�Ա��bug
	OC_Structure.TIM_OCMode=TIM_OCMode_PWM1;//���ģʽѡ�� ���� �м� pwm1 pwm2
	OC_Structure.TIM_OCPolarity=TIM_OCPolarity_High;//��ƽ���Է�ת����
	OC_Structure.TIM_OutputState=ENABLE;
	OC_Structure.TIM_Pulse = 0;//����ccr

	TIM_OC3Init(TIM1,&OC_Structure);
	TIM_OC3PolarityConfig(TIM1,TIM_OCPolarity_High);
  TIM_OC4Init(TIM1,&OC_Structure);
	TIM_OC4PolarityConfig(TIM1,TIM_OCPolarity_High);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_SetCompare3(TIM1, 1500);
    TIM_SetCompare4(TIM1, 1500);

	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//�߼���ʱ��Ĭ�������pwmͨ���ǹرյģ�Ҫ��

}

static void  	ServoElbow_Init(void)//TIM9_CH1, PA2
{
	GPIO_InitTypeDef GPIOinitstrcture = {0};
	TIM_TimeBaseInitTypeDef TIM_structure;
	TIM_OCInitTypeDef OC_Structure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_AF;
	GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_2;
	GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIOinitstrcture.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOinitstrcture.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA,&GPIOinitstrcture);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);

	TIM_InternalClockConfig(TIM9);
	TIM_structure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_structure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_structure.TIM_Period = 20000-1;
	TIM_structure.TIM_Prescaler = 168-1;
	TIM_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9,&TIM_structure);

	TIM_OCStructInit(&OC_Structure);
	OC_Structure.TIM_OCMode = TIM_OCMode_PWM1;
	OC_Structure.TIM_OCPolarity = TIM_OCPolarity_High;
	OC_Structure.TIM_OutputState = ENABLE;
	OC_Structure.TIM_Pulse = 0;

	TIM_OC1Init(TIM9,&OC_Structure);
	TIM_OC1PolarityConfig(TIM9,TIM_OCPolarity_High);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_SetCompare1(TIM9, SERVO_PULSE(ARM_HOME_ELBOW));

	TIM_Cmd(TIM9,ENABLE);
}

void PWM_init(void)//总初始化
{
     Pwm_TIM_Init();
    Servoshoulder_Init();   // TIM1 CH1/CH2(腰/大臂)+CH3/CH4(小臂PA10/夹爪)
    // CH3/CH4 已在 Servoshoulder_Init 内一并初始化，避免 ServoGripper_Init 二次 TIM_TimeBaseInit 导致 UG 抖动
    ServoElbow_Init();      // TIM9 CH1(小臂PA2)
}

void Servo_PWMEnable(uint8_t id, uint8_t enable)
{
    FunctionalState s = enable ? ENABLE : DISABLE;
    switch (id) {
        case 0: TIM_CCxCmd(TIM1, TIM_Channel_1, s); break;
        case 1: TIM_CCxCmd(TIM1, TIM_Channel_2, s); break;
        case 2: TIM_CCxCmd(TIM9, TIM_Channel_1, s); break;
        case 3: TIM_CCxCmd(TIM1, TIM_Channel_4, s); break;
        default: break;
    }
}
//����ռ�ձȣ�channelѡ���ĸ����/�����pulseռ�ձ�
void PWM_Setcompare(uint8_t channel1, uint32_t pulse)//ע��arr+1=50������pulse��Χ0-50
{
	switch(channel1 )
	{
	      case MOTOR_LEFT_FRONT:   TIM_SetCompare2(TIM3, pulse); break;  // PA7 = TIM3_CH2
        case MOTOR_RIGHT_FRONT:  TIM_SetCompare1(TIM3, pulse); break;  // PA6 = TIM3_CH1
        case MOTOR_LEFT_REAR:    TIM_SetCompare4(TIM3, pulse); break;  // PB1 = TIM3_CH4
        case MOTOR_RIGHT_REAR:   TIM_SetCompare2(TIM12, pulse); break;
        case SERVO_WAIST:        TIM_SetCompare1(TIM1, pulse); break;
        case SERVO_SHOULDER:     TIM_SetCompare2(TIM1, pulse); break;
        case SERVO_ELBOW:        TIM_SetCompare3(TIM1, pulse); break;
        case SERVO_GRIPPER:      TIM_SetCompare4(TIM1, pulse); break;
        case SERVO_ELBOW_PA2:    TIM_SetCompare1(TIM9, pulse); break;
        default: break;

	}
}
