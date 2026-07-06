#ifndef __IIC_H
#define __IIC_H
#include "sys.h"
#include "stm32f4xx.h"

/* I2C pin: PA5=SCL, PA12=SDA */
#define OLED_SCL_Clr()  GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define OLED_SCL_Set()  GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define OLED_SDA_Clr()  GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define OLED_SDA_Set()  GPIO_SetBits(GPIOA, GPIO_Pin_12)

/* SDA方向切换: 发送时输出, 读ACK时输入 */
#define SDA_OUT()  do { GPIOA->MODER = (GPIOA->MODER & ~(3 << 24)) | (1 << 24); } while(0)
#define SDA_IN()   do { GPIOA->MODER &= ~(3 << 24); } while(0)

void IIC_delay(void);
void I2C_Start(void);
void I2C_Stop(void);
u8   I2C_WaitAck(void);
void Send_Byte(u8 dat);

#endif
