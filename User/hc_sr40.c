#include "stm32f4xx.h"
#include "hc_sr40.h"
#include <string.h>

// 蓝牙环形缓冲区
static volatile uint8_t bt_rx_buf[BT_BUF_SIZE];
static volatile uint16_t bt_rx_head = 0;
static volatile uint16_t bt_rx_tail = 0;

// 临时行缓冲区
static char bt_line_buf[128];

/**
 * @brief  初始化蓝牙串口（USART2）
 */
void BT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能 GPIOD 和 USART2 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // PD5 - TX, PD6 - RX
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 复用功能配置
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // USART 配置
    USART_InitStructure.USART_BaudRate            = BT_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BT_USART, &USART_InitStructure);

    // 配置中断
    NVIC_InitStructure.NVIC_IRQChannel                   = BT_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能接收中断
    USART_ITConfig(BT_USART, USART_IT_RXNE, ENABLE);
    // 使能 UART
    USART_Cmd(BT_USART, ENABLE);
}

/**
 * @brief  发送单字节
 */
void BT_SendByte(uint8_t byte)
{
    USART_SendData(BT_USART, byte);
    while (USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  发送字符串
 */
void BT_SendString(const char *str)
{
    while (*str) {
        BT_SendByte((uint8_t)*str++);
    }
}

/**
 * @brief  串口接收中断服务函数 (蓝牙未用, 已禁用)
 */
#if 0
void BT_USART_IRQHandler(void)
{
    if (USART_GetITStatus(BT_USART, USART_IT_RXNE) == SET) {
        uint8_t ch = USART_ReceiveData(BT_USART);
        uint16_t next = (bt_rx_head + 1) % BT_BUF_SIZE;
        if (next != bt_rx_tail) {
            bt_rx_buf[bt_rx_head] = ch;
            bt_rx_head = next;
        }
    }

    // 溢出错误：TIM6(优先级1)可能阻塞USART2(优先级2)导致ORE
    // ORE不清除会让后续RXNE中断永久失效
    if (USART_GetFlagStatus(BT_USART, USART_FLAG_ORE) == SET) {
        (void)USART_ReceiveData(BT_USART);
    }
}
#endif /* 蓝牙未用, 禁用ISR避免与调试串口冲突 */

/**
 * @brief  从环形缓冲区提取一行，\n 或 \r 均可作为行结束符
 *         返回值：1 表示取到了完整一行，0 表示还没有完整行
 */
static int extract_bt_line(char *line, uint16_t max_len)
{
    uint16_t i = 0;
    while (bt_rx_tail != bt_rx_head && i < max_len - 1) {
        char ch = bt_rx_buf[bt_rx_tail];
        bt_rx_tail = (bt_rx_tail + 1) % BT_BUF_SIZE;

        if (ch == '\n' || ch == '\r') {
            line[i] = '\0';
            if (i > 0) return 1;  // 非空行返回
            i = 0;                // 空行重置，继续等待
        } else {
            line[i++] = ch;
        }
    }
    return 0;
}

/**
 * @brief  供外部调用的获取一行命令函数
 */
int BT_GetLine(char *line, uint16_t max_len)
{
    if (extract_bt_line(bt_line_buf, sizeof(bt_line_buf)) == 1) {
        // 调试回显：直接用BT_SendByte发回，不经过printf/fputc
        // 手机看到 [数据] 说明RX+TX硬件通路正常，问题在fputc
        // 手机什么都看不到说明RX或TX通路有问题
        BT_SendByte('[');
        BT_SendString(bt_line_buf);
        BT_SendByte(']');
        BT_SendByte('\r');
        BT_SendByte('\n');

        strncpy(line, bt_line_buf, max_len - 1);
        line[max_len - 1] = '\0';
        return 1;
    }
    return 0;
}
