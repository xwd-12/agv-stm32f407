#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "enconder.h"

extern int8_t encoder_dir[4];

static TIM_TypeDef* Encoder_TIM[4] = {TIM5, TIM2, TIM4, TIM8};
static int32_t last_raw_count[4] = {0, 0, 0, 0};
static int32_t position_accum[4] = {0, 0, 0, 0};

void Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_structure;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	// 释放JTAG(PA15/PB3/PB4)为GPIO，仅保留SWD(PA13/PA14)
	// STM32F4上PA15/PB3通过GPIO_PinAFConfig覆盖默认AF0即可释放
	// DBGMCU->CR只控制TRACE引脚(PE2-PE6)，与JTAG无关
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	// 左前编码器: TIM5, PA0, PA1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	// 右前编码器: TIM2, PA15, PB3
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// 左后编码器: TIM4, PB6, PB7
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	// 右后编码器: TIM8, PC6, PC7
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
        TIM5->CCMR1 |= 0xF0F0;  // IC1F=IC2F=max filter, suppress encoder noise
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, ENABLE);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM2->CCMR1 |= 0xF0F0;  // IC1F=IC2F=max filter
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM4->CCMR1 |= 0xF0F0;  // IC1F=IC2F=max filter
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);

	// TIM8高级定时器，需初始化BDTR，使能MOE后编码器才计数
	TIM_TimeBaseStructInit(&TIM_structure);
	TIM_TimeBaseInit(TIM8, &TIM_structure);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM8->CCMR1 |= 0xF0F0;  // IC1F=IC2F=max filter
	TIM_SetCounter(TIM8, 0);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
}

int16_t Encoder_getspeed(uint8_t idx)
{
	int32_t current;
	int32_t delta;
	int32_t scaled;
	int32_t max_delta;
	int32_t max_valid;
	static int16_t last_valid_speed[4] = {0,0,0,0};
	static uint8_t fault_cnt[4] = {0,0,0,0};

	if (idx >= 4)
		return 0;

	current = (int32_t)TIM_GetCounter(Encoder_TIM[idx]);
	delta = current - last_raw_count[idx];
	// 处理16位定时器计数器回绕（TIM4/TIM8），10ms内脉冲数远小于32767
	if (delta > 32767) delta -= 65536;
	if (delta < -32768) delta += 65536;

	// 数据有效性校验: 单次delta不应超过 encoder_max*3 (预留过冲余量)
	max_delta = encoder_max[idx];
	if (max_delta < 1) max_delta = 1;
	max_valid = max_delta * 10;
	if (delta > max_valid || delta < -max_valid) {
		// 异常跳变: 丢弃本次读数, 使用上次有效值, 累加故障计数
		fault_cnt[idx]++;
		if (fault_cnt[idx] > 10) {
			// 连续故障超过10次(100ms), 复位计数器
			last_raw_count[idx] = current;
			fault_cnt[idx] = 0;
		}
		return last_valid_speed[idx];
	}
	// 有效读数: 更新计数器, 重置故障计数
	fault_cnt[idx] = 0;
	last_raw_count[idx] = current;
	position_accum[idx] += delta;

	scaled = delta * 1000 / max_delta;
	if (scaled > 1000) scaled = 1000;
	if (scaled < -1000) scaled = -1000;
	last_valid_speed[idx] = (int16_t)scaled;

		// IIR低通滤波: filtered = old*0.7 + new*0.3
		{
			static int16_t filtered_speed[4] = {0,0,0,0};
			int32_t filtered;
			filtered = (int32_t)filtered_speed[idx] * 7 + (int32_t)scaled * 3;
			filtered_speed[idx] = (int16_t)(filtered / 10);
			return filtered_speed[idx];
		}
}

int32_t Encoder_GetPosition(uint8_t idx)
{
	if (idx >= 4) return 0;
	return position_accum[idx] * encoder_dir[idx];
}

void Encoder_ClearPosition(uint8_t idx)
{
	if (idx >= 4) return;
	TIM_SetCounter(Encoder_TIM[idx], 0);
	last_raw_count[idx] = 0;
	position_accum[idx] = 0;
}
