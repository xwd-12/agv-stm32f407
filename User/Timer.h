#include "stm32f4xx.h"
#ifndef __Timer_h
#define __Timer_h

extern volatile uint32_t g_sys_tick;
extern volatile int32_t line_time_ticks;

void Timer_Init(void);
void PVD_Init(void);
void TIM6_DAC_IRQHandler(void);

#endif
