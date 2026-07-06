#include "DELAY.h"
#include "stm32f4xx.h"

// ============================================================
// SysTick + 毫秒计数器（非阻塞延时系统）
// ============================================================

// 全局毫秒计数器（由 SysTick 中断每毫秒递增）
volatile uint32_t g_sys_ms = 0;

// 初始化 SysTick（1ms 中断）
void SysTick_Init(void)
{
    // STM32F4: SysTick 时钟 = HCLK = 168MHz
    // 168MHz / 168 = 1MHz => 每 168 计数 = 1ms
    // 实际上：1MHz / 1000 = 1000，所以 RELOAD = 168000 / 1000 = 168
    // 或者直接：168000000 / 1000 = 168000
    SysTick->LOAD = 168000 - 1;  // 168MHz / 168000 = 1000 Hz = 1ms
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   // HCLK = 168MHz
                  | SysTick_CTRL_TICKINT_Msk    // 使能中断
                  | SysTick_CTRL_ENABLE_Msk;    // 启动
}

// ============================================================
// 非阻塞延时 API
// ============================================================

// 获取当前系统时间（毫秒）
uint32_t Delay_GetMs(void)
{
    return g_sys_ms;
}

// 延时（阻塞式，基于计数器查询，非中断等待）
void Delay_ms(uint32_t ms)
{
    uint32_t start = g_sys_ms;
    while ((g_sys_ms - start) < ms);
}

// 微秒延时（阻塞式，粗略精确）
void Delay_us(uint32_t us)
{
    // 168MHz 下，每个 tick = 1/168 us
    // 所以 us 个微秒 = us * 168 个 tick
    uint32_t ticks = us * 168;
    if (ticks == 0) return;

    SysTick->LOAD = ticks & 0xFFFFFF;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_ENABLE_Msk;   // 不开中断，纯查询
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0;
}

// ============================================================
// 非阻塞延时辅助（适合状态机等待）
// ============================================================

// 启动一个延时（立即返回）
void Delay_Start(DelayHandle_t *h, uint32_t ms)
{
    h->start     = g_sys_ms;
    h->duration  = ms;
    h->started   = 1;
}

// 检查延时是否完成（返回 1 表示完成，0 表示还在等待）
uint8_t Delay_Check(const DelayHandle_t *h)
{
    if (!h->started) return 1;
    return ((g_sys_ms - h->start) >= h->duration) ? 1 : 0;
}

// 延时完成自动重启动（适合周期等待，返回 1 表示刚完成一次）
uint8_t Delay_CheckAndReset(DelayHandle_t *h, uint32_t ms)
{
    if (!h->started) {
        h->start    = g_sys_ms;
        h->duration = ms;
        h->started  = 1;
        return 0;
    }
    if ((g_sys_ms - h->start) >= h->duration) {
        h->start = g_sys_ms;  // 重启
        return 1;
    }
    return 0;
}

// 延时完成标志读取后清除（适合一次性事件，返回 1 表示刚完成）
uint8_t Delay_CheckAndClear(DelayHandle_t *h, uint32_t ms)
{
    if (!h->started) {
        h->start    = g_sys_ms;
        h->duration = ms;
        h->started  = 1;
        return 0;
    }
    if ((g_sys_ms - h->start) >= h->duration) {
        h->started = 0;  // 清除
        return 1;
    }
    return 0;
}
