#ifndef __line_follow_h
#define __line_follow_h
#include "stm32f4xx.h"
#define MODE_MANUAL       0
#define MODE_LINE_FOLLOW  1
#define MODE_POSITION     2
typedef struct
	{
	 int16_t base_speed ;
	 float kp  ;
	 float ki  ;
	 float kd ;
	 float integal  ;
	 float last_err  ;
	 float last_deriv ;
	 float integral_sep_th ;
	 float last_turn ;
	    /* 弯道计数抓取 */
	    uint8_t  curve_count;
	    uint8_t  curve_target;
	    uint16_t curve_delay_ms;
	    uint8_t  curve_ready;
	    uint32_t curve_done_tick;
	    uint8_t  in_curve;
	    uint8_t  curve_cooldown;
	float dt ;
	 int16_t rr_offset;  /* 右后轮开环补偿, 默认-20, 串口 rroff 命令在线调 */
		}Line_follow_Handle;

void lineFollow_Init(Line_follow_Handle*lf,int16_t speed,float kp,float ki, float kd,float dt);
void  LineFollow_setbaseSpeed(Line_follow_Handle*lf,int16_t speed);
void LineFollow_SetKp(Line_follow_Handle *lf, float kp) ;
void LineFollow_SetKi(Line_follow_Handle *lf, float ki) ;
void LineFollow_SetKd(Line_follow_Handle *lf, float kd) ;
void LineFollow_Reset(Line_follow_Handle *lf);
void LineFollow_Task(Line_follow_Handle*lf,volatile int16_t target_speed[4]);


#endif
