#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include "stm32f4xx.h"
#include "delay.h"
void LED_Init(void)
{
    // 1. 声明并立即初始化结构体，确保所有成员默认为0，防止垃圾值影响
    GPIO_InitTypeDef GPIOinitstrcture = {0};

    // 2. 使能 GPIO 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    // 3. 配置 GPIO 参数
    GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_OUT;      // 输出模式
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_9; // 引脚 1 和 2
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_NOPULL;       // 上拉
    GPIOinitstrcture.GPIO_Speed = GPIO_Medium_Speed;  // 中速
    GPIOinitstrcture.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    
    // 4. 初始化 GPIO
    GPIO_Init(GPIOF, &GPIOinitstrcture);
    
    // 5. 设置引脚为高电平（原注释“配置IO口”不准确，已修正逻辑理解，代码行为保持不变）
    GPIO_SetBits(GPIOF, GPIO_Pin_9);
		
}
void LED_set1(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1	);
}

void LED_reset1(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1	);
}
void LED_set2(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_2	);
}

void LED_reset2(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_2	);
}

void LED1_Turn()
{
	if
		(GPIO_ReadOutputDataBit(GPIOF,GPIO_Pin_9)==0)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_9);
     Delay_ms(500);
        }	
	else
		{
			GPIO_ResetBits(GPIOF,GPIO_Pin_9);
			Delay_ms(500);
}
		}