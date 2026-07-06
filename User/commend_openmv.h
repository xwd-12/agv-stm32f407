#ifndef __commend_openmv_h
#define __commend_openmv_h
#include "stm32f4xx.h"

/* OpenMV 使用的 UART */
#define OMV_USART               USART3
#define OMV_USART_IRQn          USART3_IRQn
#define OMV_USART_IRQHandler    USART3_IRQHandler

/* 波特率 */
#define OMV_BAUDRATE            115200

/* 环形缓冲区大小 */
#define OMV_BUF_SIZE            256
#define OMV_QR_TEXT_MAX         64

/* 视觉工作模式 (与 OpenMV 脚本一致) */
#define OMV_MODE_APRILTAG  0
#define OMV_MODE_COLOR     1
#define OMV_MODE_QRCODE    2
#define OMV_MODE_AI        3

/* QR 码解码数据 */
typedef struct {
    uint8_t  fresh;
    char     text[OMV_QR_TEXT_MAX];
} OpenMV_QRData;

/* OpenMV 视觉数据包 */
typedef struct {
    uint8_t  fresh;          /* 1 = 有新数据未读 */
    uint32_t timestamp;      /* g_sys_tick 快照 */
    int16_t  tag_id;         /* -1 = 未检测到标签, 0~255 = 标签ID */
    int16_t  cx;             /* 标签中心 X (0~320 图像坐标) */
    int16_t  cy;             /* 标签中心 Y (0~240 图像坐标) */
    int16_t  distance_cm;    /* 估计距离 (cm) */
    int16_t  angle_deg;      /* 标签偏角 (-180~+180) */
    int16_t  pixel_width;    /* 标签像素宽度 (用于测距) */
} OpenMV_Data;

/* 初始化 OpenMV 串口 */
void OpenMV_Init(void);

/* 发送单字节 */
void OpenMV_SendByte(uint8_t byte);

/* 发送字符串 */
void OpenMV_SendString(const char *str);

/* 主循环调用：从环形缓冲区提取一行并解析 */
void OpenMV_PollLine(void);

/* 安全清空环形缓冲区 (机械臂长阻塞后丢弃过时数据) */
void OpenMV_FlushRx(void);

/* 获取最新视觉数据 (拷贝后清除 fresh 标志), 返回 1=有效 */
int OpenMV_GetData(OpenMV_Data *out);

/* 查询是否有新数据 */
int OpenMV_IsFresh(void);

/* 查询指定标签是否在 max_age ticks 内出现过 (不消费数据) */
int OpenMV_TagSeenRecently(int16_t tag_id, uint32_t max_age);

/* 获取最近看到的标签 ID */
int16_t OpenMV_GetLastTagId(void);

/* 向 OpenMV 发送指令 */
void OpenMV_SendCmd(const char *cmd);

/* 设置视觉工作模式 */
void OpenMV_SetMode(uint8_t mode);

/* 设置目标颜色 (1=红 2=绿 3=蓝) */
void OpenMV_SetTargetColor(uint8_t color_id);

/* 一键切颜色追踪模式 + 目标颜色 (合并命令, 避免两条命令间的竞态) */
void OpenMV_SetModeColor(uint8_t color_id);

/* 设置目标标签 ID */
void OpenMV_SetTargetTag(int16_t tag_id);

/* 获取 QR 解码文本 (拷贝后清除 fresh), 返回 1=有效 */
int OpenMV_GetQRData(OpenMV_QRData *out);

/* 查询 OpenMV 心跳是否存活 (max_age 单位: 100Hz ticks) */
int OpenMV_IsAlive(uint32_t max_age);

/* AI 分类结果数据 */
typedef struct {
    uint8_t  fresh;
    int16_t  class_id;      /* -1 = 不确定, 0~N = 类别ID */
    uint8_t  confidence;    /* 0~100 置信度百分比 */
} OpenMV_CLSData;

/* 获取 AI 分类结果 (拷贝后清除 fresh), 返回 1=有效 */
int OpenMV_GetCLSData(OpenMV_CLSData *out);

/* Bug#5: 获取 OpenMV 最近错误信息 (拷贝后清除 fresh), 返回 1=有错误 */
int OpenMV_GetLastError(char *err_buf, uint16_t max_len);

/* Bug#5: 查询 AI 模型是否就绪, 返回 1=AI可用 0=未就绪 */
int OpenMV_IsAIReady(void);
/* 查询模型是否加载失败, 返回 1=加载失败 */
int OpenMV_IsModelError(void);

/* Bug#5: 清除 AI 模型错误标志, 允许下次重试加载 */
void OpenMV_ClearAIError(void);

/* 打印 OpenMV 诊断状态 (心跳/模式/数据) */
void OpenMV_PrintStatus(void);

#endif
