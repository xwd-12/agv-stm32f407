#include "DELAY.h"
#include "delay.h"
#include "stm32f4xx.h"
// 软件微秒延时（基于CPU空转，时间和系统时钟强相关）
void Delay_us(uint32_t us)
{
    uint32_t i;
    for(i = 0; i < us * 10; i++)  // 系数10是根据F407 168MHz主频估算的，可根据实际情况微调
    {
        __NOP(); // 空操作，防止编译器优化掉循环
    }
}

// 软件毫秒延时
void Delay_ms(uint32_t ms)
{
    uint32_t i;
    for(i = 0; i < ms; i++)
    {
        Delay_us(1000);
    }
}