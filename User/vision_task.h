#ifndef __vision_task_h
#define __vision_task_h
#include "stm32f4xx.h"

/* 视觉任务状态 */
typedef enum {
    VIS_IDLE = 0,
    VIS_COLOR_SEARCH,       /* 巡线+颜色扫描 */
    VIS_COLOR_APPROACH,     /* 接近检测到的色块 */
    VIS_COLOR_GRASP,        /* 抓取色块 */
    VIS_QR_SEARCH,          /* 巡线+二维码扫描 */
    VIS_QR_APPROACH,        /* 接近二维码 */
    VIS_QR_EXECUTE,         /* 执行二维码指令 */
    VIS_AI_SCAN,            /* AI 分类扫描 */
    VIS_AI_APPROACH,        /* 接近检测到的目标 */
    VIS_AI_EXECUTE,         /* 执行 AI 分类结果 */
    VIS_DONE,
    VIS_ERROR
} VisionTaskState;

/* 视觉任务控制句柄 */
typedef struct {
    VisionTaskState state;
    uint8_t  target_color;      /* 目标颜色 1=红 2=绿 3=蓝 */
    uint32_t state_start_tick;  /* 状态开始时间 */
    uint32_t check_tick;        /* 检测节流 */
    uint32_t lost_tick;         /* 目标丢失开始时间 */
    uint8_t  lost_timeout;      /* 丢失超时 (100Hz ticks, 默认 100 = 1s) */
    int16_t  approach_dist;     /* 接近距离阈值 (cm) */
    int16_t  grasp_dist;        /* 抓取距离阈值 (cm) */
    int16_t  ai_class_id;       /* AI 分类结果 */
    uint8_t  ai_confidence;     /* AI 置信度 */
    char     qr_command[64];    /* QR 指令文本 */
} VisionTask_Handle;

/* 初始化 */
void VisionTask_Init(VisionTask_Handle *vt);

/* 启动颜色搜索任务 */
void VisionTask_StartColorSearch(VisionTask_Handle *vt, uint8_t color);

/* 启动二维码搜索任务 */
void VisionTask_StartQRSearch(VisionTask_Handle *vt);

/* 启动 AI 分类扫描 */
void VisionTask_StartAIScan(VisionTask_Handle *vt);

/* 停止任务 */
void VisionTask_Stop(VisionTask_Handle *vt);

/* 主循环调用：非阻塞状态机 */
void VisionTask_Process(VisionTask_Handle *vt);

/* 查询状态 */
VisionTaskState VisionTask_GetState(const VisionTask_Handle *vt);

/* 状态名 */
const char* VisionTask_StateName(VisionTaskState s);

/* 查询是否忙 */
uint8_t VisionTask_IsBusy(const VisionTask_Handle *vt);

#endif
