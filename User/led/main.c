/**
  *
  * 实验平台：SICV  STM32_F407 开发板
	*
  * 开发部门：产学研事业体技术部
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "./led/bsp_led.h"
#include "key.h"
#include "Delay.h"
uint8_t KEYNUM;
int main(void)
{
	
	LED_Init( );
	key_init( );
	while (1) 
	{
	KEYNUM  = keyget( );
		LED_Set(KEYNUM,KEYNUM);
		if(KEYNUM == 1)
		LED1_TUrn();
		if (KEYNUM == 2)
		LED2_TUrn();
	}
}

/****************************************END OF FILE**********************/

