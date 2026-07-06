#include "stm32f4xx.h"
#ifndef  __uart_h
#define  __uart_h
void uart_Init(void);
void uart_sendbyte(uint8_t Byte);
void uart_sendchar( const char *str);//const表示只读，不可修改
void UART5_IRQHandler(void);
int uart_rb_getchar(void);
int UART_GetCommand(char *cmd_buf, uint16_t max_len);
int uart_ring_peek(const char *pattern);
void uart_set_bt_output(int enable);


#endif
