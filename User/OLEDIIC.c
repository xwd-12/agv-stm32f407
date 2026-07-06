#include "OLEDIIC.h"
#include "stm32f4xx.h"

void IIC_delay(void)
{
    u32 t;
    t = 280;
    while (t--);
}

void I2C_Start(void)
{
    SDA_OUT();
    OLED_SDA_Set();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Clr();
    IIC_delay();
    OLED_SCL_Clr();
    IIC_delay();
}

void I2C_Stop(void)
{
    SDA_OUT();
    OLED_SDA_Clr();
    IIC_delay();
    OLED_SCL_Set();
    IIC_delay();
    OLED_SDA_Set();
    IIC_delay();
}

u8 I2C_WaitAck(void)
{
    u8 ack;
    u16 timeout;

    SDA_IN();
    OLED_SDA_Set();
    IIC_delay();
    OLED_SCL_Set();
    IIC_delay();

    timeout = 5000;
    while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) && --timeout);

    ack = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);

    OLED_SCL_Clr();
    IIC_delay();
    SDA_OUT();
    return ack;
}

void Send_Byte(u8 dat)
{
    u8 i;

    SDA_OUT();
    for (i = 0; i < 8; i++) {
        if (dat & 0x80)
            OLED_SDA_Set();
        else
            OLED_SDA_Clr();
        IIC_delay();
        OLED_SCL_Set();
        IIC_delay();
        OLED_SCL_Clr();
        dat <<= 1;
    }
}
