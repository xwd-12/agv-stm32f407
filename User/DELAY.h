#ifndef _DELAY_H
#define _DELAY_H
#include "stm32f4xx.h"

// ============================================================
// 初始化（启动 SysTick 1ms 定时器中断）
// ============================================================
void SysTick_Init(void);

// 获取当前系统时间（毫秒）
uint32_t Delay_GetMs(void);

// ============================================================
// 阻塞式延时
// ============================================================
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

// ============================================================
// 非阻塞延时（适合状态机）
// ============================================================

// 延时句柄结构体
typedef struct {
    uint32_t start;
    uint32_t duration;
    uint8_t  started;
} DelayHandle_t;

// 启动一个延时（立即返回，不阻塞）
void Delay_Start(DelayHandle_t *h, uint32_t ms);

// 检查延时是否完成（返回 1=完成, 0=还在等）
uint8_t Delay_Check(const DelayHandle_t *h);

// 延时完成自动重启动（适合周期等待，返回 1 表示刚完成一次）
uint8_t Delay_CheckAndReset(DelayHandle_t *h, uint32_t ms);

// 延时完成标志读取后清除（适合一次性事件，返回 1 表示刚完成）
uint8_t Delay_CheckAndClear(DelayHandle_t *h, uint32_t ms);
#endif
