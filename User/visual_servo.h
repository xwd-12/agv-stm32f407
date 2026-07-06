#ifndef __visual_servo_h
#define __visual_servo_h
#include "stm32f4xx.h"
#include "commend_openmv.h"

/* 视觉伺服工作模式 */
#define MODE_VISUAL_SERVO  3

/* 视觉伺服控制句柄 (仿 line_follow.h 的 Line_follow_Handle 模式) */
typedef struct
{
    /* 控制目标 */
    int16_t target_tag_id;       /* 目标 AprilTag ID (-1 = 任意标签) */
    int16_t target_distance_cm;  /* 期望停靠距离 (cm) */
    int16_t target_cx;           /* 期望标签 X 中心 (默认 160, 320x240 图像) */

    /* 速度限幅 */
    int16_t base_speed_max;      /* 接近时最大前进速度 */
    int16_t turn_limit;          /* 差速转向最大差值 */

    /* 横向 PID (X 方向对准, 控制转向) */
    float kp_lat;
    float ki_lat;
    float kd_lat;
    float integal_lat;
    float last_err_lat;
    float last_deriv_lat;

    /* 纵向 PID (距离控制, 控制前进/后退) */
    float kp_long;
    float ki_long;
    float kd_long;
    float integal_long;
    float last_err_long;
    float last_deriv_long;

    /* 通用 */
    float integral_sep_th;
    float dt;

    /* 对准判定 */
    uint8_t aligned;             /* 1 = 对接完成 */
    uint8_t lost_cnt;            /* 连续丢失标签的帧数 */
    uint8_t lost_threshold;      /* 丢失多少帧后判定为丢失 (10 = 100ms) */
    int16_t aligned_threshold_x; /* 横向对准容差 (像素) */
    int16_t aligned_threshold_d; /* 纵向对准容差 (cm) */
    uint8_t aligned_frames;      /* 连续对准帧数 */
    uint8_t aligned_required;    /* 需要多少连续帧才判定对准 (默认 5 = 50ms) */
} VisualServo_Handle;

/* 初始化 */
void VisualServo_Init(VisualServo_Handle *vs,
    int16_t target_tag_id, int16_t target_dist_cm,
    int16_t base_speed, float kp_lat, float ki_lat, float kd_lat,
    float kp_long, float ki_long, float kd_long, float dt);

/* 设置目标 */
void VisualServo_SetTarget(VisualServo_Handle *vs,
    int16_t tag_id, int16_t dist_cm);

/* 设置横向 PID */
void VisualServo_SetLatPID(VisualServo_Handle *vs,
    float kp, float ki, float kd);

/* 设置纵向 PID */
void VisualServo_SetLongPID(VisualServo_Handle *vs,
    float kp, float ki, float kd);

/* 重置 PID 状态 */
void VisualServo_Reset(VisualServo_Handle *vs);

/* 视觉伺服主任务 (ISR 中 100Hz 调用) */
void VisualServo_Task(VisualServo_Handle *vs,
    const OpenMV_Data *omv, volatile int16_t target_speed[4]);

/* 查询是否对准 */
int VisualServo_IsAligned(const VisualServo_Handle *vs);

/* 查询是否丢失标签 */
int VisualServo_IsLost(const VisualServo_Handle *vs);

/* 设置对准阈值 */
void VisualServo_SetAlignThreshold(VisualServo_Handle *vs,
    int16_t x_px, int16_t d_cm, uint8_t frames);

/* 设置丢失阈值 */
void VisualServo_SetLostThreshold(VisualServo_Handle *vs,
    uint8_t threshold);

#endif
