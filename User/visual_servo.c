#include "stm32f4xx.h"
#include "visual_servo.h"

/**
 * @brief  初始化视觉伺服句柄
 */
void VisualServo_Init(VisualServo_Handle *vs,
    int16_t target_tag_id, int16_t target_dist_cm,
    int16_t base_speed, float kp_lat, float ki_lat, float kd_lat,
    float kp_long, float ki_long, float kd_long, float dt)
{
    vs->target_tag_id      = target_tag_id;
    vs->target_distance_cm = target_dist_cm;
    vs->target_cx          = 160;       /* 320x240 图像的水平中心 */
    vs->base_speed_max     = base_speed;
    vs->turn_limit         = 400;       /* 差速限幅 */

    /* 横向 PID */
    vs->kp_lat = kp_lat;
    vs->ki_lat = ki_lat;
    vs->kd_lat = kd_lat;
    vs->integal_lat   = 0.0f;
    vs->last_err_lat  = 0.0f;
    vs->last_deriv_lat = 0.0f;

    /* 纵向 PID */
    vs->kp_long = kp_long;
    vs->ki_long = ki_long;
    vs->kd_long = kd_long;
    vs->integal_long   = 0.0f;
    vs->last_err_long  = 0.0f;
    vs->last_deriv_long = 0.0f;

    /* 通用 */
    vs->integral_sep_th = 60.0f;        /* 横向像素误差 > 60 时积分清零 */
    vs->dt = dt;

    /* 对准判定 */
    vs->aligned              = 0;
    vs->lost_cnt             = 0;
    vs->lost_threshold       = 20;      /* 20 帧 = 200ms 无标签判定丢失 */
    vs->aligned_threshold_x  = 10;      /* 横向 ±10 像素内视作对准 */
    vs->aligned_threshold_d  = 3;       /* 纵向 ±3cm 内视作对准 */
    vs->aligned_frames       = 0;
    vs->aligned_required     = 5;       /* 连续 5 帧对准判定稳定 */
}

/**
 * @brief  设置目标标签和距离
 */
void VisualServo_SetTarget(VisualServo_Handle *vs,
    int16_t tag_id, int16_t dist_cm)
{
    vs->target_tag_id      = tag_id;
    vs->target_distance_cm = dist_cm;
    vs->aligned            = 0;
    vs->aligned_frames     = 0;
}

/**
 * @brief  设置横向 PID 参数
 */
void VisualServo_SetLatPID(VisualServo_Handle *vs,
    float kp, float ki, float kd)
{
    vs->kp_lat = kp;
    vs->ki_lat = ki;
    vs->kd_lat = kd;
}

/**
 * @brief  设置纵向 PID 参数
 */
void VisualServo_SetLongPID(VisualServo_Handle *vs,
    float kp, float ki, float kd)
{
    vs->kp_long = kp;
    vs->ki_long = ki;
    vs->kd_long = kd;
}

/**
 * @brief  设置对准阈值
 */
void VisualServo_SetAlignThreshold(VisualServo_Handle *vs,
    int16_t x_px, int16_t d_cm, uint8_t frames)
{
    vs->aligned_threshold_x = x_px;
    vs->aligned_threshold_d = d_cm;
    vs->aligned_required    = frames;
}

/**
 * @brief  设置丢失阈值 (连续多少帧无标签判定丢失)
 */
void VisualServo_SetLostThreshold(VisualServo_Handle *vs,
    uint8_t threshold)
{
    vs->lost_threshold = threshold;
}

/**
 * @brief  重置 PID 积分和微分状态
 */
void VisualServo_Reset(VisualServo_Handle *vs)
{
    vs->integal_lat    = 0.0f;
    vs->last_err_lat   = 0.0f;
    vs->last_deriv_lat = 0.0f;
    vs->integal_long    = 0.0f;
    vs->last_err_long   = 0.0f;
    vs->last_deriv_long = 0.0f;
    vs->aligned         = 0;
    vs->aligned_frames  = 0;
    vs->lost_cnt        = 0;
}

/**
 * @brief  查询是否对准
 */
int VisualServo_IsAligned(const VisualServo_Handle *vs)
{
    return (vs->aligned != 0) ? 1 : 0;
}

/**
 * @brief  查询是否丢失标签
 */
int VisualServo_IsLost(const VisualServo_Handle *vs)
{
    return (vs->lost_cnt > vs->lost_threshold) ? 1 : 0;
}

/**
 * @brief  视觉伺服主任务 (ISR 中 100Hz 调用, 仿 LineFollow_Task 模式)
 *
 *         omv == NULL 表示本周期无新数据，仅更新丢失计数
 *         omv->tag_id == -1 表示 OpenMV 未检测到任何标签
 *         横向 PID 控制转向 (turn)，纵向 PID 控制前进/后退速度
 *         差速合成: left = forward + turn, right = forward - turn
 */
void VisualServo_Task(VisualServo_Handle *vs,
    const OpenMV_Data *omv, volatile int16_t target_speed[4])
{
    float error_lat;
    float error_long;
    float derivative;
    float turn;
    float forward;
    int16_t left;
    int16_t right;

    /* 无新数据：增加丢失计数，保持当前速度不变 */
    if (omv == ((const OpenMV_Data *)0))
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        return;
    }

    /* 标签丢失 (OpenMV 未检测到任何标签) */
    if (omv->tag_id < 0)
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        /* 连续丢失超过阈值：减速停止 */
        if (vs->lost_cnt > vs->lost_threshold)
        {
            target_speed[0] = 0;
            target_speed[1] = 0;
            target_speed[2] = 0;
            target_speed[3] = 0;
            vs->aligned = 0;
            vs->aligned_frames = 0;
        }
        return;
    }

    /* 标签 ID 不匹配 (指定了目标标签且当前标签不是目标) */
    if (vs->target_tag_id >= 0 && omv->tag_id != vs->target_tag_id)
    {
        /* 看到错误标签: 不停止, 不增加丢失计数, 仅忽略 */
        /* target_speed 保持上个周期的值继续漂移, 等待正确标签出现 */
        return;
    }

    /* 有效标签：清零丢失计数 */
    vs->lost_cnt = 0;

    /* ===== 横向 PID (X 中心对准 → 转向控制) ===== */
    error_lat = (float)(vs->target_cx - omv->cx);

    /* 积分分离 */
    if (error_lat > vs->integral_sep_th || error_lat < -vs->integral_sep_th)
        vs->integal_lat = 0.0f;

    /* 梯形积分 */
    vs->integal_lat += (error_lat + vs->last_err_lat) * 0.5f * vs->dt;
    if (vs->integal_lat > 200.0f)  vs->integal_lat = 200.0f;
    if (vs->integal_lat < -200.0f) vs->integal_lat = -200.0f;

    /* 微分 + IIR 低通滤波 */
    derivative = (error_lat - vs->last_err_lat) / vs->dt;
    vs->last_deriv_lat = 0.3f * vs->last_deriv_lat + 0.7f * derivative;
    derivative = vs->last_deriv_lat;

    /* 横向 PID 输出 */
    turn = vs->kp_lat * error_lat + vs->ki_lat * vs->integal_lat
           + vs->kd_lat * derivative;
    vs->last_err_lat = error_lat;

    /* 限幅 */
    if (turn > (float)vs->turn_limit)  turn = (float)vs->turn_limit;
    if (turn < (float)(-vs->turn_limit)) turn = (float)(-vs->turn_limit);

    /* ===== 纵向 PID (距离控制 → 前进/后退速度) ===== */
    error_long = (float)(vs->target_distance_cm - omv->distance_cm);

    /* 积分分离: 距离误差 > 30cm 时清零积分 */
    if (error_long > 30.0f || error_long < -30.0f)
        vs->integal_long = 0.0f;

    /* 梯形积分 */
    vs->integal_long += (error_long + vs->last_err_long) * 0.5f * vs->dt;
    if (vs->integal_long > 200.0f)  vs->integal_long = 200.0f;
    if (vs->integal_long < -200.0f) vs->integal_long = -200.0f;

    /* 微分 + IIR 低通滤波 */
    derivative = (error_long - vs->last_err_long) / vs->dt;
    vs->last_deriv_long = 0.3f * vs->last_deriv_long + 0.7f * derivative;
    derivative = vs->last_deriv_long;

    /* 纵向 PID 输出 */
    forward = vs->kp_long * error_long + vs->ki_long * vs->integal_long
              + vs->kd_long * derivative;
    vs->last_err_long = error_long;

    /* 限幅 */
    if (forward > (float)vs->base_speed_max)  forward = (float)vs->base_speed_max;
    if (forward < (float)(-vs->base_speed_max)) forward = (float)(-vs->base_speed_max);

    /* ===== 差速合成 ===== */
    left  = (int16_t)(forward + turn);
    right = (int16_t)(forward - turn);

    /* 限幅 ±1000 */
    if (left > 1000)   left = 1000;
    if (left < -1000)  left = -1000;
    if (right > 1000)  right = 1000;
    if (right < -1000) right = -1000;

    target_speed[0] = left;
    target_speed[1] = right;
    target_speed[2] = left;
    target_speed[3] = right;

    /* ===== 对准判定 ===== */
    {
        float abs_lat;
        float abs_long;
        if (error_lat < 0.0f) abs_lat = -error_lat; else abs_lat = error_lat;
        if (error_long < 0.0f) abs_long = -error_long; else abs_long = error_long;

        if (abs_lat < (float)vs->aligned_threshold_x &&
            abs_long < (float)vs->aligned_threshold_d)
        {
            if (vs->aligned_frames < 255)
                vs->aligned_frames++;
            if (vs->aligned_frames >= vs->aligned_required)
                vs->aligned = 1;
        }
        else
        {
            vs->aligned_frames = 0;
            vs->aligned = 0;
        }
    }
}
