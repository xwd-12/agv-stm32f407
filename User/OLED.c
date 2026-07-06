#include "OLED.h"
#include "OLEDIIC.h"
#include "word.h"
#include "DELAY.h"

u8 OLED_GRAM[144][8];

void OLED_WR_Byte(u8 dat, u8 mode)
{
    I2C_Start();
    Send_Byte(0x78);
    I2C_WaitAck();
    if (mode)
        Send_Byte(0x40);
    else
        Send_Byte(0x00);
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

void OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    u8 i, n;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIOA->AFR[0] &= ~(0xF << (5 * 4));
    GPIOA->AFR[1] &= ~(0xF << ((12 - 8) * 4));

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_12);

    Delay_ms(100);

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x30, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);

    Delay_ms(100);

    OLED_WR_Byte(0xAF, OLED_CMD);

    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0;

    OLED_Refresh();
}

void OLED_Refresh(void)
{
    u8 i, n;

    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        I2C_Start();
        Send_Byte(0x78);
        I2C_WaitAck();
        Send_Byte(0x40);
        I2C_WaitAck();
        for (n = 0; n < 128; n++)
        {
            Send_Byte(OLED_GRAM[n][i]);
            I2C_WaitAck();
        }
        I2C_Stop();
    }
}

void OLED_Clear(void)
{
    u8 i, n;

    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0;

    OLED_Refresh();
}

void OLED_ColorTurn(u8 i)
{
    if (i == 0)
        OLED_WR_Byte(0xA6, OLED_CMD);
    if (i == 1)
        OLED_WR_Byte(0xA7, OLED_CMD);
}

void OLED_DisplayTurn(u8 i)
{
    if (i == 0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if (i == 1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_DisPlay_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    u8 i, m, n;

    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t)
        OLED_GRAM[x][i] |= n;
    else
        OLED_GRAM[x][i] &= ~n;
}

void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 mode)
{
    u16 t;
    int xerr, yerr, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    xerr = 0;
    yerr = 0;
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0)
        incx = 1;
    else if (delta_x == 0)
        incx = 0;
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0;
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (t = 0; t < (u16)(distance + 1); t++)
    {
        OLED_DrawPoint(uRow, uCol, mode);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void OLED_DrawCircle(u8 x, u8 y, u8 r)
{
    int a, b, num;

    a = 0;
    b = r;
    while (2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);
        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);

        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0)
        {
            b--;
            a--;
        }
    }
}

void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1, u8 mode)
{
    u8 i, m, temp, size2, chr1;
    u8 x0, y0;

    x0 = x;
    y0 = y;
    if (size1 == 8)
        size2 = 6;
    else
        size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1 = chr - ' ';

    for (i = 0; i < size2; i++)
    {
        if (size1 == 8)
            temp = asc2_0806[chr1][i];
        else if (size1 == 12)
            temp = asc2_1206[chr1][i];
        else if (size1 == 16)
            temp = asc2_1608[chr1][i];
        else if (size1 == 24)
            temp = asc2_2412[chr1][i];
        else
            return;

        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((size1 != 8) && ((x - x0) == size1 / 2))
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1, u8 mode)
{
    while ((*chr >= ' ') && (*chr <= '~'))
    {
        OLED_ShowChar(x, y, *chr, size1, mode);
        if (size1 == 8)
            x += 6;
        else
            x += size1 / 2;
        chr++;
    }
    OLED_Refresh();
}

u32 OLED_Pow(u8 m, u8 n)
{
    u32 result;

    result = 1;
    while (n--)
        result *= m;
    return result;
}

void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1, u8 mode)
{
    u8 t, temp, m;

    m = 0;
    if (size1 == 8)
        m = 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if (temp == 0)
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        else
            OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
    }
}

void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1, u8 mode)
{
    u8 m, temp;
    u8 x0, y0;
    u16 i, size3;

    x0 = x;
    y0 = y;
    size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;
    for (i = 0; i < size3; i++)
    {
        if (size1 == 16)
            temp = Hzk1[num][i];
        else if (size1 == 24)
            temp = Hzk2[num][i];
        else if (size1 == 32)
            temp = Hzk3[num][i];
        else if (size1 == 64)
            temp = Hzk4[num][i];
        else
            return;

        for (m = 0; m < 8; m++)
        {
            if (temp & 0x01)
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if ((x - x0) == size1)
        {
            x = x0;
            y0 = y0 + 8;
        }
        y = y0;
    }
}

void OLED_ScrollDisplay(u8 num, u8 space, u8 mode)
{
    u8 i, n, t, m, r;

    t = 0;
    m = 0;
    while (1)
    {
        if (m == 0)
        {
            OLED_ShowChinese(128, 24, t, 16, mode);
            t++;
        }
        if (t == num)
        {
            for (r = 0; r < 16 * space; r++)
            {
                for (i = 1; i < 144; i++)
                {
                    for (n = 0; n < 8; n++)
                    {
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                    }
                }
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16)
            m = 0;

        for (i = 1; i < 144; i++)
        {
            for (n = 0; n < 8; n++)
            {
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
            }
        }
        OLED_Refresh();
    }
}

void OLED_ShowPicture(u8 x, u8 y, u8 sizex, u8 sizey, u8 BMP[], u8 mode)
{
    u16 j;
    u8 i, n, temp, m;
    u8 x0, y0;

    j = 0;
    x0 = x;
    y0 = y;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for (n = 0; n < sizey; n++)
    {
        for (i = 0; i < sizex; i++)
        {
            temp = BMP[j];
            j++;
            for (m = 0; m < 8; m++)
            {
                if (temp & 0x01)
                    OLED_DrawPoint(x, y, mode);
                else
                    OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == sizex)
            {
                x = x0;
                y0 = y0 + 8;
            }
            y = y0;
        }
    }
}

float My_abs(float a)
{
    if (a >= 0)
        return a;
    else
        return -a;
}

void OLED_ShowFloat(u8 x, u8 y, float num, u8 len, u8 size, u8 mode)
{
    float Abs_num;
    int unit, tens, hundreds, one_decimal, two_decimal;

    Abs_num = My_abs(num);
    hundreds = (int)Abs_num / 100;
    tens = ((int)Abs_num / 10) % 10;
    unit = Abs_num - hundreds * 100 - tens * 10;
    one_decimal = (int)((Abs_num - (int)Abs_num) * 10);
    two_decimal = (int)((Abs_num - (int)Abs_num) * 100) % 10;

    if (num >= 0)
    {
        OLED_ShowChar(x, y, '+', size, mode);
        OLED_ShowNum(x + 8, y, hundreds, 1, size, mode);
        OLED_ShowNum(x + 16, y, tens, 1, size, mode);
        OLED_ShowNum(x + 24, y, unit, 1, size, mode);
        OLED_ShowChar(x + 32, y, '.', size, mode);
        OLED_ShowNum(x + 40, y, one_decimal, 1, size, mode);
        OLED_ShowNum(x + 48, y, two_decimal, 1, size, mode);
    }
    else
    {
        OLED_ShowChar(x, y, '-', size, mode);
        OLED_ShowNum(x + 8, y, hundreds, 1, size, mode);
        OLED_ShowNum(x + 16, y, tens, 1, size, mode);
        OLED_ShowNum(x + 24, y, unit, 1, size, mode);
        OLED_ShowChar(x + 32, y, '.', size, mode);
        OLED_ShowNum(x + 40, y, one_decimal, 1, size, mode);
        OLED_ShowNum(x + 48, y, two_decimal, 1, size, mode);
    }
    OLED_Refresh();
}
