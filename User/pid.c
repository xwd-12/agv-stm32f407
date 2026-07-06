#include "stm32f4xx.h"
#include "pid.h"
#include "math.h"

#define PID_DEFAULT_DT  0.01f  // 默认10ms周期

void PID_Reset(PID_Handle *pid)
{
    pid->integral       = 0.0f;
    pid->last_err       = 0.0f;
    pid->last_feedback  = 0.0f;
    pid->last_out       = 0.0f;
    pid->last2_err      = 0.0f;
    pid->last_deriv     = 0.0f;
    pid->initialized    = 0;
}

void PID_Init(PID_Handle *pid, PID_Type type,
              float Kp, float Ki, float Kd,
              float integral_limit, float out_limit, float integral_sep_th)
{
    pid->type              = type;
    pid->Kp               = Kp;
    pid->Ki               = Ki;
    pid->Kd               = Kd;
    pid->integral_limit   = integral_limit;
    pid->out_limit        = out_limit;
    pid->integral_sep_th  = integral_sep_th;
    PID_Reset(pid);
}

/*
 * PID_Update: 统一的位置式/增量式PID更新
 *
 * 抗积分饱和策略:
 * 1. 积分分离: |error| > integral_sep_th 时积分清零
 * 2. 条件积分: 输出饱和时停止同向积分累积 (back-calculation)
 * 3. 积分限幅: integral = clamp(integral, ±integral_limit)
 * 4. 输出限幅: out = clamp(out, ±out_limit)
 *
 * 微分项: 采用微分先行(位置式) 或 低通滤波(通用)
 *          减少测量噪声对微分项的放大
 */
float PID_Update(PID_Handle *pid, float setpoint, float feedback, float dt)
{
    float error = setpoint - feedback;
    float out   = 0.0f;
    float derivative;
float delta;
    // dt 有效性检查
    if (dt < 1e-6f)
        dt = PID_DEFAULT_DT;

    // 首次调用初始化
    if (!pid->initialized) {
        pid->last_feedback = feedback;
        pid->initialized   = 1;
    }

    // ------- 积分分离: 大偏差时清零积分 -------
    if (fabsf(error) > pid->integral_sep_th) {
        pid->integral = 0.0f;
    }

    // ------- 位置式PID -------
    if (pid->type == PID_POSITION) {

        // 微分先行: 对反馈值微分，避免设定值阶跃冲击
        derivative = (pid->last_feedback - feedback) / dt;

        // 低通滤波微分项 (一阶IIR, alpha=0.7 即 3dB截止≈0.35/Ts)
        pid->last_deriv = 0.3f * pid->last_deriv + 0.7f * derivative;
        derivative = pid->last_deriv;

        // 梯形积分
        if (pid->Ki != 0.0f) {
            float clamped_out;
            float delta_integral = (error + pid->last_err) * 0.5f * dt;
            float trial_integral = pid->integral + delta_integral;

            // 条件积分: 输出饱和且误差同向时停止积分
            clamped_out = pid->Kp * error + pid->Ki * trial_integral + pid->Kd * derivative;
            if (clamped_out > pid->out_limit && error > 0.0f) {
                // 正向饱和, 不累加正向积分
            } else if (clamped_out < -pid->out_limit && error < 0.0f) {
                // 负向饱和, 不累加负向积分
            } else {
                pid->integral = trial_integral;
            }
        }

        // 积分限幅
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;

        out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

        pid->last_err      = error;
        pid->last_feedback = feedback;

    // ------- 增量式PID -------
    } else {

        derivative = (error - pid->last_err) / dt;

        // 低通滤波微分
        pid->last_deriv = 0.3f * pid->last_deriv + 0.7f * derivative;
        derivative = pid->last_deriv;

         delta = pid->Kp * (error - pid->last_err)
                    + pid->Ki * error * dt
                    + pid->Kd * (error - 2.0f * pid->last_err + pid->last2_err) / dt;

        out = pid->last_out + delta;

        // 输出限幅
        if (out > pid->out_limit)
            out = pid->out_limit;
        if (out < -pid->out_limit)
            out = -pid->out_limit;

        pid->last2_err = pid->last_err;
        pid->last_out  = out;
        pid->last_err  = error;
    }

    // ------- 输出限幅 -------
    if (out > pid->out_limit)  out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;

    return out;
}
