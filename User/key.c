#include "stm32f4xx.h"
#include "DELAY.h"
void key_init(void)
{
	GPIO_InitTypeDef GPIOinitstructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIOinitstructure.GPIO_Mode = GPIO_Mode_IN;
	GPIOinitstructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIOinitstructure.GPIO_Speed =GPIO_Medium_Speed ;
	GPIOinitstructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIOinitstructure);
}

uint8_t keyget(void)//配置按下按键的动作函数
{
     uint8_t keynum = 0;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))
	{
		Delay_ms(20);  // 消抖延时 20ms
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1));
		keynum = 1;
	}
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11))
	{
		Delay_ms(20);  // 消抖延时 20ms
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11));
		keynum = 2;
	}
	return keynum;
}
