#ifndef  __hc_sr40_h
#define  __hc_sr40_h
#include "stm32f4xx.h"

// ����ģ��ʹ�õ� UART
#define BT_USART               USART2
#define BT_USART_IRQn          USART2_IRQn
#define BT_USART_IRQHandler    USART2_IRQHandler

// �����ʣ���Ҫ������ģ�������һ�£�
#define BT_BAUDRATE            9600

// ��������С
#define BT_BUF_SIZE            256

// ��ʼ����������
void BT_Init(void);

// ����һ���ֽ�
void BT_SendByte(uint8_t byte);

// �����ַ���
void BT_SendString(const char *str);

// �ӻ��λ�������ȡһ������� \n ��β��
int BT_GetLine(char *line, uint16_t max_len);

#endif
