#include "stm32f4xx.h"
#include "vision_task.h"
#include "commend_openmv.h"
#include "action_group.h"
#include "state_machine.h"
#include "line_follow.h"
#include "motor.h"
#include "smooth_servo.h"
#include <string.h>
#include <stdio.h>

/* 外部变量 */
extern volatile uint8_t  work_mode;
extern volatile int16_t  target_speed[4];
extern Line_follow_Handle line_follow;
extern volatile uint8_t motor_enable;
extern volatile uint32_t g_sys_tick;

/* 默认参数 */
#define DEFAULT_APPROACH_DIST   15      /* 接近触发距离 (cm) */
#define DEFAULT_GRASP_DIST      8       /* 抓取触发距离 (cm) */
#define DEFAULT_LOST_TIMEOUT    100     /* 目标丢失超时 (100Hz ticks = 1s) */
#define CHECK_INTERVAL          5       /* 检测节流 (ticks = 50ms) */
#define APPROACH_BASE_SPEED     100     /* 接近时基础速度 */
#define APPROACH_TURN_GAIN      5       /* 接近时转向增益 */
#define WAIST_ADJUST_GAIN       0.12f   /* 腰座调整增益 (deg/px) */
#define WAIST_ADJUST_DEADBAND   8       /* 腰座居中对齐死区 (px) */
#define WAIST_ADJUST_TIME       100     /* 腰座单次调整时间 (ms) */
#define APPROACH_SPEED_SLOW     60      /* AI 接近低速 */
#define APPROACH_DIST_DEADBAND  2       /* 距离死区 (cm) */

/**
 * @brief  初始化视觉任务句柄
 */
void VisionTask_Init(VisionTask_Handle *vt)
{
    vt->state            = VIS_IDLE;
    vt->target_color     = 1;
    vt->state_start_tick = 0;
    vt->check_tick       = 0;
    vt->lost_tick        = 0;
    vt->lost_timeout     = DEFAULT_LOST_TIMEOUT;
    vt->approach_dist    = DEFAULT_APPROACH_DIST;
    vt->grasp_dist       = DEFAULT_GRASP_DIST;
    vt->ai_class_id      = -1;
    vt->ai_confidence    = 0;
    vt->qr_command[0]    = '\0';
}

/**
 * @brief  启动颜色搜索
 */
void VisionTask_StartColorSearch(VisionTask_Handle *vt, uint8_t color)
{
    int i;
    if (vt->state != VIS_IDLE)
        return;

    vt->target_color     = color;
    vt->state            = VIS_COLOR_SEARCH;
    vt->state_start_tick = g_sys_tick;
    vt->check_tick       = g_sys_tick;
    vt->lost_tick        = 0;

    /* 设置 OpenMV 为颜色追踪模式 + 目标颜色 (合并命令, 避免竞态) */
    OpenMV_SetModeColor(color);

    /* 启动巡线 */
    LineFollow_Reset(&line_follow);
    work_mode = MODE_LINE_FOLLOW;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;

    printf("Vision: color search started, color=%d\r\n", color);
}

/**
 * @brief  启动二维码搜索
 */
void VisionTask_StartQRSearch(VisionTask_Handle *vt)
{
    int i;
    if (vt->state != VIS_IDLE)
        return;

    vt->state            = VIS_QR_SEARCH;
    vt->state_start_tick = g_sys_tick;
    vt->check_tick       = g_sys_tick;
    vt->lost_tick        = 0;
    vt->qr_command[0]    = '\0';

    /* 设置 OpenMV 为二维码模式 */
    OpenMV_SetMode(OMV_MODE_QRCODE);

    /* 启动巡线 */
    LineFollow_Reset(&line_follow);
    work_mode = MODE_LINE_FOLLOW;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;

    printf("Vision: QR search started\r\n");
}

/**
 * @brief  启动 AI 分类扫描
 */
void VisionTask_StartAIScan(VisionTask_Handle *vt)
{
    int i;
    if (vt->state != VIS_IDLE)
        return;

    vt->state            = VIS_AI_SCAN;
    vt->state_start_tick = g_sys_tick;
    vt->check_tick       = 0;
    vt->ai_class_id      = -1;
    vt->ai_confidence    = 0;

    /* 切换 OpenMV 到 AI 模式 */
    OpenMV_SetMode(OMV_MODE_AI);

    /* 停车 */
    work_mode = MODE_MANUAL;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;

    printf("Vision: AI scan started\r\n");
}

/**
 * @brief  停止视觉任务
 */
void VisionTask_Stop(VisionTask_Handle *vt)
{
    int i;
    vt->state = VIS_IDLE;
    work_mode = MODE_MANUAL;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;
    printf("Vision: task stopped\r\n");
}

/**
 * @brief  查询状态
 */
VisionTaskState VisionTask_GetState(const VisionTask_Handle *vt)
{
    return vt->state;
}

/**
 * @brief  查询是否忙
 */
uint8_t VisionTask_IsBusy(const VisionTask_Handle *vt)
{
    if (vt->state != VIS_IDLE && vt->state != VIS_DONE && vt->state != VIS_ERROR)
        return 1;
    return 0;
}

/**
 * @brief  状态名
 */
const char* VisionTask_StateName(VisionTaskState s)
{
    switch (s) {
    case VIS_IDLE:           return "IDLE";
    case VIS_COLOR_SEARCH:   return "COLOR_SEARCH";
    case VIS_COLOR_APPROACH: return "COLOR_APPROACH";
    case VIS_COLOR_GRASP:    return "COLOR_GRASP";
    case VIS_QR_SEARCH:      return "QR_SEARCH";
    case VIS_QR_APPROACH:    return "QR_APPROACH";
    case VIS_QR_EXECUTE:     return "QR_EXECUTE";
    case VIS_AI_SCAN:        return "AI_SCAN";
    case VIS_AI_APPROACH:    return "AI_APPROACH";
    case VIS_AI_EXECUTE:     return "AI_EXECUTE";
    case VIS_DONE:           return "DONE";
    case VIS_ERROR:          return "ERROR";
    default:                 return "???";
    }
}

/**
 * @brief  视觉任务主状态机 (主循环调用, 非阻塞)
 */
void VisionTask_Process(VisionTask_Handle *vt)
{
    OpenMV_Data omv;
    int has_data;

    if (vt->state == VIS_IDLE || vt->state == VIS_DONE || vt->state == VIS_ERROR)
        return;

    /* 检测节流 */
    if (g_sys_tick - vt->check_tick < CHECK_INTERVAL)
        return;
    vt->check_tick = g_sys_tick;

    has_data = OpenMV_GetData(&omv);

    switch (vt->state)
    {

    /* ===== 颜色搜索 ===== */
    case VIS_COLOR_SEARCH:
    {
        int i;
        if (has_data && omv.tag_id == (int16_t)vt->target_color)
        {
            /* 检测到目标颜色 */
            vt->lost_tick = 0;

            if (omv.distance_cm > 0 && omv.distance_cm < vt->approach_dist)
            {
                /* 进入接近阶段 */
                work_mode = MODE_MANUAL;
                for (i = 0; i < 4; i++)
                    target_speed[i] = 0;
                vt->state = VIS_COLOR_APPROACH;
                vt->state_start_tick = g_sys_tick;
                printf("Vision: color %d detected at %d cm, approaching\r\n",
                       vt->target_color, (int)omv.distance_cm);
            }
        }
        else
        {
            /* 未检测到目标: 累计丢失计数 */
            if (vt->lost_tick == 0)
                vt->lost_tick = g_sys_tick;
            else if (g_sys_tick - vt->lost_tick > vt->lost_timeout)
            {
                vt->state = VIS_ERROR;
                printf("Vision: color search timeout\r\n");
            }
        }
        return;
    }

    /* ===== 颜色接近 ===== */
    case VIS_COLOR_APPROACH:
    {
        int16_t left;
        int16_t right;
        int16_t turn;
        int16_t forward;
        int i;

        if (has_data && omv.tag_id == (int16_t)vt->target_color)
        {
            vt->lost_tick = 0;

            if (omv.distance_cm > 0 && omv.distance_cm <= vt->grasp_dist)
            {
                /* 到达抓取位置 */
                for (i = 0; i < 4; i++)
                    target_speed[i] = 0;
                Motor_SetSpeed(0, 0);
                Motor_SetSpeed(1, 0);
                Motor_SetSpeed(2, 0);
                Motor_SetSpeed(3, 0);
                vt->state = VIS_COLOR_GRASP;
                vt->state_start_tick = g_sys_tick;
                printf("Vision: at grasp position, grasping\r\n");
                return;
            }

            /* 转向校正: blob cx 偏离中心时转向 */
            turn = (int16_t)((float)(omv.cx - 160) * (float)APPROACH_TURN_GAIN);
            if (turn > 300)  turn = 300;
            if (turn < -300) turn = -300;

            forward = APPROACH_BASE_SPEED;

            left  = forward + turn;
            right = forward - turn;
            if (left > 600)   left = 600;
            if (left < -600)  left = -600;
            if (right > 600)  right = 600;
            if (right < -600) right = -600;

            target_speed[0] = left;
            target_speed[1] = right;
            target_speed[2] = left;
            target_speed[3] = right;
        }
        else
        {
            /* 目标丢失 */
            if (vt->lost_tick == 0)
                vt->lost_tick = g_sys_tick;
            else if (g_sys_tick - vt->lost_tick > vt->lost_timeout)
            {
                int j;
                for (j = 0; j < 4; j++)
                    target_speed[j] = 0;
                vt->state = VIS_ERROR;
                printf("Vision: lost target during approach\r\n");
            }
        }
        return;
    }

    /* ===== 颜色抓取 ===== */
    case VIS_COLOR_GRASP:
    {
        /* 执行抓取 (阻塞在 main loop, 安全) */
        motor_enable = 0;
        ArmSM_RequestGrasp();
        Action_Grasp();
        ArmSM_NotifyComplete();
        motor_enable = 1;
        vt->state = VIS_DONE;
        printf("Vision: color grasp complete\r\n");
        return;
    }

    /* ===== 二维码搜索 ===== */
    case VIS_QR_SEARCH:
    {
        OpenMV_QRData qr;
        if (has_data && omv.tag_id == 0)
        {
            /* 检测到二维码 */
            vt->lost_tick = 0;

            if (omv.distance_cm > 0 && omv.distance_cm < vt->approach_dist)
            {
                /* 读取 QR 文本 */
                int j;
                if (OpenMV_GetQRData(&qr))
                {
                    uint16_t k;
                    k = 0;
                    while (qr.text[k] != '\0' && k < 63)
                    {
                        vt->qr_command[k] = qr.text[k];
                        k++;
                    }
                    vt->qr_command[k] = '\0';
                    printf("Vision: QR text: %s\r\n", vt->qr_command);
                }
                else
                {
                    vt->qr_command[0] = '\0';
                }

                /* 进入接近阶段 */
                work_mode = MODE_MANUAL;
                for (j = 0; j < 4; j++)
                    target_speed[j] = 0;
                vt->state = VIS_QR_APPROACH;
                vt->state_start_tick = g_sys_tick;
                printf("Vision: QR detected at %d cm, approaching\r\n",
                       (int)omv.distance_cm);
            }
        }
        else
        {
            if (vt->lost_tick == 0)
                vt->lost_tick = g_sys_tick;
            else if (g_sys_tick - vt->lost_tick > vt->lost_timeout)
            {
                vt->state = VIS_ERROR;
                printf("Vision: QR search timeout\r\n");
            }
        }
        return;
    }

    /* ===== 二维码接近 ===== */
    case VIS_QR_APPROACH:
    {
        int16_t left;
        int16_t right;
        int16_t turn;
        int16_t forward;
        int i;

        if (has_data && omv.tag_id == 0)
        {
            vt->lost_tick = 0;

            if (omv.distance_cm > 0 && omv.distance_cm <= vt->grasp_dist)
            {
                for (i = 0; i < 4; i++)
                    target_speed[i] = 0;
                vt->state = VIS_QR_EXECUTE;
                vt->state_start_tick = g_sys_tick;
                printf("Vision: at QR, executing command\r\n");
                return;
            }

            turn = (int16_t)((float)(omv.cx - 160) * (float)APPROACH_TURN_GAIN);
            if (turn > 300)  turn = 300;
            if (turn < -300) turn = -300;

            forward = APPROACH_BASE_SPEED;

            left  = forward + turn;
            right = forward - turn;
            if (left > 600)   left = 600;
            if (left < -600)  left = -600;
            if (right > 600)  right = 600;
            if (right < -600) right = -600;

            target_speed[0] = left;
            target_speed[1] = right;
            target_speed[2] = left;
            target_speed[3] = right;
        }
        else
        {
            if (vt->lost_tick == 0)
                vt->lost_tick = g_sys_tick;
            else if (g_sys_tick - vt->lost_tick > vt->lost_timeout)
            {
                int j;
                for (j = 0; j < 4; j++)
                    target_speed[j] = 0;
                vt->state = VIS_ERROR;
                printf("Vision: lost QR during approach\r\n");
            }
        }
        return;
    }

    /* ===== 二维码指令执行 ===== */
    case VIS_QR_EXECUTE:
    {
        int i;
        motor_enable = 0;

        /* 解析 QR 指令 */
        if (vt->qr_command[0] != '\0')
        {
            printf("Vision: executing QR cmd: %s\r\n", vt->qr_command);

            if (strstr(vt->qr_command, "GRASP") != ((void *)0))
            {
                ArmSM_RequestGrasp();
                Action_Grasp();
                ArmSM_NotifyComplete();
            }
            else if (strstr(vt->qr_command, "PLACE") != ((void *)0))
            {
                ArmSM_RequestPlace(110);
                Action_Place(110);
                ArmSM_NotifyComplete();
            }
        }
        else
        {
            /* 无反导指令: 默认抓取 */
            ArmSM_RequestGrasp();
            Action_Grasp();
            ArmSM_NotifyComplete();
        }

        motor_enable = 1;
        for (i = 0; i < 4; i++)
            target_speed[i] = 0;
        vt->state = VIS_DONE;
        printf("Vision: QR execute complete\r\n");
        return;
    }

    /* ===== AI 分类扫描 ===== */
    case VIS_AI_SCAN:
    {
        OpenMV_CLSData cls;
        if (OpenMV_GetCLSData(&cls))
        {
            vt->ai_class_id   = cls.class_id;
            vt->ai_confidence = cls.confidence;
            if (cls.class_id >= 0)
            {
                printf("Vision: AI classified as %d (conf=%d%%)\r\n",
                       (int)cls.class_id, (int)cls.confidence);
                /* 有位置数据: 根据距离决定直接抓还是先接近 */
                if (has_data && omv.distance_cm > 0
                    && omv.distance_cm <= vt->grasp_dist)
                {
                    vt->state = VIS_AI_EXECUTE;
                    vt->state_start_tick = g_sys_tick;
                }
                else if (has_data && omv.distance_cm > 0)
                {
                    int k;
                    work_mode = MODE_MANUAL;
                    for (k = 0; k < 4; k++)
                        target_speed[k] = 0;
                    vt->state = VIS_AI_APPROACH;
                    vt->state_start_tick = g_sys_tick;
                    vt->lost_tick = 0;
                    printf("Vision: AI target at %d cm, approaching\r\n",
                           (int)omv.distance_cm);
                }
                else
                {
                    vt->state = VIS_AI_EXECUTE;
                    vt->state_start_tick = g_sys_tick;
                }
            }
            else
            {
                printf("Vision: AI uncertain (conf=%d%%)\r\n",
                       (int)cls.confidence);
                vt->state = VIS_DONE;
            }
        }
        else
        {
            /* AI 模式超时: 10秒无结果 */
            if (g_sys_tick - vt->state_start_tick > 1000)
            {
                vt->state = VIS_ERROR;
                printf("Vision: AI scan timeout\r\n");
            }
        }
        return;
    }

    /* ===== AI 接近 ===== */
    case VIS_AI_APPROACH:
    {
        int16_t error_x;
        int16_t error_d;
        int16_t speed_cmd;
        int i;

        if (has_data && omv.distance_cm > 0)
        {
            vt->lost_tick = 0;

            /* 计算误差 */
            error_x = omv.cx - 160;
            error_d = omv.distance_cm - (int16_t)vt->grasp_dist;

            /* 对准判定: 居中 + 到位 → 抓取 */
            {
                int16_t abs_x;
                int16_t abs_d;
                if (error_x < 0) abs_x = -error_x; else abs_x = error_x;
                if (error_d < 0) abs_d = -error_d; else abs_d = error_d;

                if (abs_x <= WAIST_ADJUST_DEADBAND
                    && abs_d <= APPROACH_DIST_DEADBAND)
                {
                    for (i = 0; i < 4; i++)
                        target_speed[i] = 0;
                    Motor_SetSpeed(0, 0);
                    Motor_SetSpeed(1, 0);
                    Motor_SetSpeed(2, 0);
                    Motor_SetSpeed(3, 0);
                    vt->state = VIS_AI_EXECUTE;
                    vt->state_start_tick = g_sys_tick;
                    printf("Vision: AI aligned, grasping\r\n");
                    return;
                }
            }

            /* ---- 1. 腰座旋转对准 (横向) ---- */
            {
                int16_t abs_x;
                if (error_x < 0) abs_x = -error_x; else abs_x = error_x;

                if (abs_x > WAIST_ADJUST_DEADBAND
                    && !smoothservo_IsBusy(0))
                {
                    float current;
                    float delta;
                    float new_angle;
                    current = smooth_GetCurrentAngle(0);
                    delta   = (float)error_x * WAIST_ADJUST_GAIN;
                    new_angle = current + delta;
                    if (new_angle < 0.0f)   new_angle = 0.0f;
                    if (new_angle > 180.0f) new_angle = 180.0f;
                    smooth_Settarget(0, (uint16_t)new_angle, WAIST_ADJUST_TIME);
                }
            }

            /* ---- 2. 车身前后移动 (纵向) ---- */
            speed_cmd = 0;
            if (error_d > APPROACH_DIST_DEADBAND)
            {
                /* 太远 → 前进 */
                speed_cmd = APPROACH_SPEED_SLOW;
            }
            else if (error_d < -APPROACH_DIST_DEADBAND)
            {
                /* 太近 → 后退 */
                speed_cmd = -APPROACH_SPEED_SLOW;
            }

            for (i = 0; i < 4; i++)
                target_speed[i] = speed_cmd;
        }
        else
        {
            /* 目标丢失 */
            {
                int j;
                for (j = 0; j < 4; j++)
                    target_speed[j] = 0;
            }
            if (vt->lost_tick == 0)
                vt->lost_tick = g_sys_tick;
            else if (g_sys_tick - vt->lost_tick > vt->lost_timeout)
            {
                vt->state = VIS_ERROR;
                printf("Vision: lost AI target during approach\r\n");
            }
        }
        return;
    }

    /* ===== AI 分类执行 ===== */
    case VIS_AI_EXECUTE:
    {
        int i;
        motor_enable = 0;

        printf("Vision: AI class=%d, executing action\r\n",
               (int)vt->ai_class_id);

        /* 根据分类 ID 执行动作 */
        if (vt->ai_class_id >= 0)
        {
            ArmSM_RequestGrasp();
            Action_Grasp();
            ArmSM_NotifyComplete();
        }

        motor_enable = 1;
        for (i = 0; i < 4; i++)
            target_speed[i] = 0;
        vt->state = VIS_DONE;
        printf("Vision: AI execute complete\r\n");
        return;
    }

    default:
        return;
    }
}
