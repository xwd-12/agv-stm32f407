#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#define RX_BUF_SIZE 512

/* UART5: PC12=TX, PD2=RX */
#define DEBUG_USART             UART5
#define DEBUG_USART_IRQn        UART5_IRQn
#define DEBUG_USART_IRQHandler  UART5_IRQHandler

static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static char cmd_line[64];

void uart_Init(void)
{
    GPIO_InitTypeDef GPIOinitstrcture;
    USART_InitTypeDef usart_structure;
    NVIC_InitTypeDef NVIC_structure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    /* PC12 = UART5_TX */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_12;
    GPIOinitstrcture.GPIO_Mode  = GPIO_Mode_AF;
    GPIOinitstrcture.GPIO_OType = GPIO_OType_PP;
    GPIOinitstrcture.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIOinitstrcture);

    /* PD2 = UART5_RX */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIOinitstrcture.GPIO_Pin   = GPIO_Pin_2;
    GPIOinitstrcture.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIOinitstrcture);

    usart_structure.USART_BaudRate            = 115200;
    usart_structure.USART_WordLength          = USART_WordLength_8b;
    usart_structure.USART_StopBits            = USART_StopBits_1;
    usart_structure.USART_Parity              = USART_Parity_No;
    usart_structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_structure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USART, &usart_structure);

    USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(DEBUG_USART, ENABLE);

    NVIC_structure.NVIC_IRQChannel                   = DEBUG_USART_IRQn;
    NVIC_structure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_structure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_structure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_structure);
}

void uart_sendbyte(uint8_t Byte)
{
    USART_SendData(DEBUG_USART, Byte);
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

void uart_sendchar(const char *str)
{
    while (*str)
        uart_sendbyte((uint8_t)*str++);
}

extern void BT_SendByte(uint8_t byte);

static int g_printf_to_bt = 0;

void uart_set_bt_output(int enable)
{
    g_printf_to_bt = enable;
}

int fputc(int ch, FILE *f)
{
    if (g_printf_to_bt)
        BT_SendByte((uint8_t)ch);
    else
        uart_sendbyte((uint8_t)ch);
    return ch;
}

void DEBUG_USART_IRQHandler(void)
{
    if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) == SET)
    {
        uint16_t next;
        uint8_t ch;

        if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_FE) != RESET ||
            USART_GetFlagStatus(DEBUG_USART, USART_FLAG_NE) != RESET)
        {
            (void)USART_ReceiveData(DEBUG_USART);
        }
        else
        {
            ch = (uint8_t)USART_ReceiveData(DEBUG_USART);
            next = (rx_head + 1) % RX_BUF_SIZE;
            if (next != rx_tail)
            {
                rx_buf[rx_head] = ch;
                rx_head = next;
            }
        }
    }
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) == SET)
    {
        (void)USART_ReceiveData(DEBUG_USART);
    }
}

static int extract_line(char *line, uint16_t max_len)
{
    uint16_t i;
    char ch;

    i = 0;
    while (rx_tail != rx_head && i < max_len - 1)
    {
        ch = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        if (ch == '\n' || ch == '\r')
        {
            line[i] = '\0';
            if (i > 0) return 1;
            i = 0;
        }
        else
        {
            line[i++] = ch;
        }
    }
    return 0;
}

int uart_rb_getchar(void)
{
    uint8_t ch;
    if (rx_tail == rx_head) return -1;
    ch = (uint8_t)rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return (int)ch;
}

int UART_GetCommand(char *cmd_buf, uint16_t max_len)
{
    if (extract_line(cmd_line, sizeof(cmd_line)) == 1)
    {
        strncpy(cmd_buf, cmd_line, max_len - 1);
        cmd_buf[max_len - 1] = '\0';
        return 1;
    }
    return 0;
}

/*
 * 在环形缓冲区中查找子串, 只读不消费 (用于紧急停止检测)
 * 返回 1=找到, 0=未找到
 * 匹配不区分大小写
 */
int uart_ring_peek(const char *pattern)
{
    uint16_t pos;
    uint16_t len;
    uint16_t pi;
    uint16_t pi2;
    uint8_t match;

    /* 先算 pattern 长度 */
    len = 0;
    while (pattern[len] != '\0')
        len++;
    if (len == 0)
        return 0;

    /* 环形缓冲区里有多少字节 */
    {
        uint16_t avail;
        if (rx_head >= rx_tail)
            avail = rx_head - rx_tail;
        else
            avail = RX_BUF_SIZE - rx_tail + rx_head;
        if (avail < len)
            return 0;
    }

    /* 遍历每个起始位置 */
    pos = rx_tail;
    while (pos != rx_head) {
        /* 从 pos 开始匹配 len 个字符 */
        match = 1;
        pi = pos;
        for (pi2 = 0; pi2 < len; pi2++) {
            char c;
            c = rx_buf[pi];
            /* 不区分大小写 */
            if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
            if (c != pattern[pi2]) {
                match = 0;
                break;
            }
            pi++;
            if (pi >= RX_BUF_SIZE)
                pi = 0;
            /* 别追尾 */
            if (pi == rx_head && pi2 + 1 < len) {
                match = 0;
                break;
            }
        }
        if (match)
            return 1;
        pos++;
        if (pos >= RX_BUF_SIZE)
            pos = 0;
    }
    return 0;
}
