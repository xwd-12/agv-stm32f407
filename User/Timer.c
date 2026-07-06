#include "timer.h"
#include "stm32f4xx.h"
#include "enconder.h"
#include "motor.h"
#include "pid.h"
#include "line_follow.h"
#include "smooth_servo.h"
#include "state_machine.h"
#include "commend_openmv.h"
#include "visual_servo.h"
/* 视觉伺服已启用 — VisualServo_Handle 声明见下方 extern */
 extern VisualServo_Handle visual_servo;
extern PID_Handle pid_motor[4];
extern volatile int16_t target_speed[4];
extern int8_t motor_dir[4];
extern int8_t encoder_dir[4];
extern volatile uint8_t motor_enable;
extern PID_Handle pid_positon[4];
extern int32_t target_position[4] ;
extern volatile uint8_t work_mode;
extern Line_follow_Handle line_follow;

volatile uint32_t g_sys_tick = 0;
volatile int32_t line_time_ticks = 0;   // 巡线定时停止: 存储截止 tick (非倒计数)
int16_t last_pid_setpoint[4] = {0};
int16_t last_pid_speed[4] = {0};
int16_t last_pid_output[4] = {0};
int16_t last_pid_error[4] = {0};

void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef Tim_Struture;
	NVIC_InitTypeDef NVIC_Struture;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

	TIM_InternalClockConfig(TIM6);
	Tim_Struture.TIM_ClockDivision =TIM_CKD_DIV1 ;
	Tim_Struture.TIM_CounterMode =TIM_CounterMode_Up ;
	Tim_Struture.TIM_Prescaler = 8400-1;
	Tim_Struture.TIM_Period =100-1 ;
	Tim_Struture.TIM_RepetitionCounter =0 ;
	TIM_TimeBaseInit(TIM6,&Tim_Struture);

	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);

	TIM_ClearFlag(TIM6,TIM_IT_Update);

	NVIC_Struture.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_Struture.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Struture.NVIC_IRQChannelPreemptionPriority =1 ;
	NVIC_Struture.NVIC_IRQChannelSubPriority =0;
	NVIC_Init(&NVIC_Struture);

	TIM_Cmd(TIM6,ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
	int i = 0;
	static uint8_t pos_step = 0;
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)
	{
		g_sys_tick++;
		smoothservo_TimerHandle();

				ArmSM_Tick10ms();

		if(work_mode == MODE_LINE_FOLLOW)
		{
			if (line_time_ticks > 0 && g_sys_tick >= (uint32_t)line_time_ticks) {
				for(i = 0; i < 4; i++)
					target_speed[i] = 0;
				work_mode = MODE_MANUAL;
				line_time_ticks = 0;
			}
			LineFollow_Task(&line_follow,target_speed);
		}
		else if(work_mode == MODE_POSITION)
		{
			pos_step++;
			if ( pos_step>= 2)
			{
				pos_step = 0;
				for(i = 0;i<4;i++)
				{
					int32_t current_pos = Encoder_GetPosition(i);
					float speed_cmd = PID_Update(&pid_positon[i],(float)target_position[i],(float) current_pos,0.02f);
					if(speed_cmd>600) speed_cmd= 600.0f;
					if(speed_cmd<-600) speed_cmd=-600.0f;
					target_speed[i] = (int16_t)speed_cmd;
				}
			}
		}
		else if (work_mode == MODE_VISUAL_SERVO)
		{
			OpenMV_Data omv;
			if (OpenMV_GetData(&omv))
				VisualServo_Task(&visual_servo, &omv, target_speed);
			else
				VisualServo_Task(&visual_servo, (const OpenMV_Data *)0, target_speed);
		}


		// speed closed loop 10ms
		if (motor_enable)
		{
			for(i = 0; i < 4; i++)
			{
				int16_t out;
				int16_t real;
				int16_t sp;
				if (encoder_max[i] == 0) {
					// 开环模式: 编码器损坏，直接映射target_speed
					sp = motor_dir[i] * target_speed[i];
					out = sp;
					if (out > 1000) out = 1000;
					if (out < -1000) out = -1000;
					real = 0;
				} else {
					real = Encoder_getspeed(i) * encoder_dir[i];
					sp = motor_dir[i] * target_speed[i];
					out = (int16_t)PID_Update(&pid_motor[i], (float)sp, (float)real, 0.01f);
					if (out > 1000) out = 1000;
					if (out < -1000) out = -1000;
				}
				Motor_SetSpeed(i, out);
				last_pid_setpoint[i] = sp;
				last_pid_speed[i] = real;
				last_pid_output[i] = out;
				last_pid_error[i] = (int16_t)(sp - real);
			}
		}
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}
		

void PVD_Init(void)
{
	NVIC_InitTypeDef NVIC_Struture;
	EXTI_InitTypeDef EXTI_Struture;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	PWR_PVDLevelConfig(PWR_PVDLevel_7);  // 2.9V threshold
	PWR_PVDCmd(ENABLE);

	EXTI_Struture.EXTI_Line = EXTI_Line16;
	EXTI_Struture.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Struture.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Struture.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_Struture);

	NVIC_Struture.NVIC_IRQChannel = PVD_IRQn;
	NVIC_Struture.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Struture.NVIC_IRQChannelSubPriority = 0;
	NVIC_Struture.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Struture);
}

void PVD_IRQHandler(void)
{
	int i;
	if (EXTI_GetITStatus(EXTI_Line16) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line16);
		for (i = 0; i < 4; i++)
		{
			Motor_SetSpeed(i, 0);
		}
		motor_enable = 0;
	}
}
