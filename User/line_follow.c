#include "stm32f4xx.h"
#include "line_follow.h"
#include "line_sensor.h"

extern volatile uint32_t g_sys_tick;


void lineFollow_Init(Line_follow_Handle*lf,int16_t speed,float kp,float ki, float kd,float dt)
{
   lf->base_speed =speed;
	lf->kp = kp;
	lf->ki = ki;
	lf->kd = kd;
  lf->dt = dt;
  lf->integal = 0.0f;
  lf->last_err = 0.0f;
  lf->last_deriv = 0.0f;
  lf->integral_sep_th = 1.5f;
  lf->last_turn = 0.0f;
	lf->rr_offset = -20;
	lf->curve_count = 0;
	lf->curve_target = 4;
	lf->curve_delay_ms = 2000;
	lf->curve_ready = 0;
	lf->curve_done_tick = 0;
	lf->in_curve = 0;
	lf->curve_cooldown = 0;
}

void  LineFollow_setbaseSpeed(Line_follow_Handle*lf,int16_t speed)
{
    lf->base_speed = speed;
}

void LineFollow_SetKp(Line_follow_Handle *lf, float kp) { lf->kp = kp; }
void LineFollow_SetKi(Line_follow_Handle *lf, float ki) { lf->ki = ki; }
void LineFollow_SetKd(Line_follow_Handle *lf, float kd) { lf->kd = kd; }

void LineFollow_Reset(Line_follow_Handle *lf)
{
    lf->integal = 0.0f;
    lf->last_err = 0.0f;
    lf->last_deriv = 0.0f;
    lf->last_turn = 0.0f;
}

void LineFollow_Task(Line_follow_Handle*lf,volatile int16_t target_speed[4])
{
	bool states[5];
	uint8_t i, detected;
	float error;
	int16_t left;
	int16_t right;
	float turn;
	float derivative;
		static uint8_t lost_cnt = 0;
		static int16_t last_left = 0;
		static int16_t last_right = 0;
		static float last_turn_dir = 0.0f;

		LineSensor_Read(states);
	detected = 0;
	for (i = 0; i < 5; i++) {
		if (states[i]) detected++;
	}

		if (detected == 0) {
			lost_cnt++;
			if (lost_cnt > 50) {
				lf->integal = 0.0f;
				lf->last_err = 0.0f;
				lf->last_turn = 0.0f;
				last_left = 0;
				last_right = 0;
				last_turn_dir = 0.0f;
				target_speed[0] = 0;
				target_speed[1] = 0;
				target_speed[2] = 0;
				target_speed[3] = 0;
			} else {
				int16_t coast_l, coast_r;
				coast_l = (int16_t)((float)last_left * 0.6f);
				coast_r = (int16_t)((float)last_right * 0.6f);
				if (coast_l < 30 && coast_l > -30) coast_l = (last_left >= 0) ? 30 : -30;
				if (coast_r < 30 && coast_r > -30) coast_r = (last_right >= 0) ? 30 : -30;
				last_left = coast_l;
				last_right = coast_r;
				target_speed[0] = coast_l;
				target_speed[1] = coast_r;
				target_speed[2] = coast_l;
				target_speed[3] = coast_r;
			}
			return;
		}
		lost_cnt = 0;
	/* sancha fork (left+right on, center off) -> default turn LEFT, once per junction */
	{
		static uint8_t fork_hold = 0;
		static uint8_t fork_tick = 0;
		static uint8_t fork_armed = 1;
		uint8_t left_on, right_on;
		left_on  = states[3] || states[4];
		right_on = states[0] || states[1];
		/* Ö»ÔÚÖ±ÐÐÖÐ´¥·¢(×î½üÎó²î¡Ö0): ÍäµÀÖÐ²»´¥·¢, PIDÕý³£×ªÍä */
		if (left_on && right_on && !states[2]
		    && last_turn_dir > -0.5f && last_turn_dir < 0.5f) {
			if (fork_armed) {
				fork_armed = 0;
				fork_hold = 1;
				fork_tick = 0;
			}
		} else {
			fork_armed = 1;   /* Àë¿ªÂ·¿Ú¸´Î», ÏÂ¸öÂ·¿Ú¿ÉÔÙ´¥·¢ */
		}
		if (fork_hold) {
			fork_tick++;
			target_speed[0] = 120;
			target_speed[1] = 500;
			target_speed[2] = 120;
			target_speed[3] = 470;
			if (fork_tick >= 40 || states[2]) {
				fork_hold = 0;
			}
			return;
		}
	}


	error = LineSensor_CalcError(states);
		/* å¼¯é“æ£€æµ? â‰?ä¸ªä¼ æ„Ÿå™¨æ£€æµ‹åˆ°çº¿ä¸”ä¿æŒ3å¸§åˆ¤å®šä¸ºå¼¯é“ */
			{
				static uint8_t curve_db = 0;
				if (detected <= 4) {
					if (curve_db < 3) curve_db++;
					if (curve_db >= 3 && !lf->in_curve && lf->curve_cooldown == 0) {
						lf->in_curve = 1;
						lf->curve_count++;
						if (lf->curve_target > 0 && lf->curve_count >= lf->curve_target) {
							lf->curve_done_tick = g_sys_tick;
						}
					}
				} else {
					curve_db = 0;
					if (lf->in_curve) {
						lf->in_curve = 0;
						lf->curve_cooldown = 50;
					}
				}
				if (lf->curve_cooldown > 0) lf->curve_cooldown--;
			}
		/* åˆ°è¾¾ç›®æ ‡å¼¯é“+å»¶æ—¶åŽç½®ä½?*/
		if (lf->curve_target > 0 && lf->curve_count >= lf->curve_target && lf->curve_done_tick > 0) {
			if (g_sys_tick - lf->curve_done_tick >= (uint32_t)(lf->curve_delay_ms / 10)) {
				lf->curve_ready = 1;
			}
		}

		/* deadband on RAW error: prevent EMA pollution from previous curves */
		if (error < 0.1f && error > -0.1f) {
			error = 0.0f;
			lf->integal = 0.0f;
		}



	/* integral separation + trapezoidal integration */
	if (error > lf->integral_sep_th || error < -lf->integral_sep_th) {
		lf->integal = 0.0f;
	}
	lf->integal += (error + lf->last_err) * 0.5f * lf->dt;
	if (lf->integal > 30.0f) lf->integal = 30.0f;
	if (lf->integal < -30.0f) lf->integal = -30.0f;

	derivative = (error - lf->last_err) / lf->dt;
	lf->last_deriv = 0.7f * lf->last_deriv + 0.3f * derivative;
	derivative = lf->last_deriv;

	turn = lf->kp * error + lf->ki * lf->integal + lf->kd * derivative;
	lf->last_err = error;

	if (turn > 600.0f) turn = 600.0f;
	if (turn < -600.0f) turn = -600.0f;

	left  = (int16_t)((float)lf->base_speed + turn);
	right = (int16_t)((float)lf->base_speed - turn);

	if (left > 1000)  left = 1000;
	if (left < 0)     left = 0;
	if (right > 1000) right = 1000;
	if (right < 0)    right = 0;

	last_left = left;
	last_right = right;
		last_turn_dir = error;

		target_speed[0] = left;
		target_speed[1] = right;
		/* ×óºóÂÖ¹ÕÍä²¹³¥: ×ªÍäÊ±¶à¸øÁ¦(ÓÃ»§: ¹ÕÍä×ª²»ÆðÀ´) */
		if (turn > 20.0f || turn < -20.0f) {
			target_speed[2] = left + 30;
			if (target_speed[2] > 1000) target_speed[2] = 1000;
		} else {
			target_speed[2] = left;
		}
	/* å³åŽç¼–ç å™¨å, å¼€çŽ¯åå¿? å•ç‹¬å‡?0è¡¥å¿ */
	{
		int16_t rr;
		rr = right + lf->rr_offset;
		if (rr < 0) rr = 0;
			if (rr > 1000) rr = 1000;
		target_speed[3] = rr;
	}
}
