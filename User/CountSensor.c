#include "stm32f4xx.h"

void CountSensor_Init(void)
{
		GPIO_InitTypeDef GPIO_Initstruture;
	EXTI_InitTypeDef EXTI_Initstruture;
	NVIC_InitTypeDef NVIC_structure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//开启gpio和afio的时钟

	GPIO_Initstruture.GPIO_Mode=GPIO_Mode_IN;
	GPIO_Initstruture.GPIO_OType=GPIO_OType_PP;
	GPIO_Initstruture.GPIO_PuPd =GPIO_PuPd_UP ;
	GPIO_Initstruture.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB,&GPIO_Initstruture);//初始化gpiob(有前面用了结构体进行配置才有初始化函数进行初始化)
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource1);//AFIO的中断引脚配置
	
	EXTI_Initstruture.EXTI_Line=EXTI_Line1;//中断线
	EXTI_Initstruture.EXTI_LineCmd=ENABLE;
	EXTI_Initstruture.EXTI_Mode = EXTI_Mode_Interrupt;//进行中断响应
	EXTI_Initstruture.EXTI_Trigger =EXTI_Trigger_Rising_Falling ;
	EXTI_Init(&EXTI_Initstruture);//EXTI的配置与初始化
	

	
	NVIC_structure.NVIC_IRQChannel=EXTI1_IRQn;//中断通道
	NVIC_structure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_structure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级
	NVIC_structure.NVIC_IRQChannelSubPriority = 3;//响应优先级
	NVIC_Init(&NVIC_structure);//NVIC的配置与初始化
}

void EXTI1_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line1) == SET)
	 {
	 }
	 EXTI_ClearITPendingBit(EXTI_Line1);
}