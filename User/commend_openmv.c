#include "stm32f4xx.h"
#include "commend_openmv.h"
#include <string.h>
#include <stdio.h>

extern volatile uint32_t g_sys_tick;

/* OpenMV 颜色环形缓冲区 */
static volatile uint8_t omv_rx_buf[OMV_BUF_SIZE];
static volatile uint16_t omv_rx_head = 0;
static volatile uint16_t omv_rx_tail = 0;

/* OpenMV 数据单例 */
static OpenMV_Data omv_data;
static int16_t  omv_last_tag_id = -1;    /* 最近看到的标签 ID (不被消费清除) */
static uint32_t omv_last_tag_tick = 0;   /* 最近看到标签的时间戳 */

/* QR 码文本环形缓冲区 */
static volatile char  omv_qr_buf[OMV_QR_TEXT_MAX];
static volatile uint8_t omv_qr_fresh = 0;

/* AI 分类结果 */
static volatile OpenMV_CLSData omv_cls_data;
static volatile uint8_t omv_cls_fresh = 0;

/* 心跳 */
static volatile uint32_t omv_last_hb_tick = 0;

/* Bug#5: 错误信息存储 (AI 模型加载失败等) */
static volatile uint8_t  omv_error_fresh = 0;
static volatile char     omv_error_text[64] = "";
static volatile uint8_t  omv_ai_ready = 0;     /* 收到 $OK,BOOT 且无模型错误时置1 */
static volatile uint8_t  omv_model_error = 0;  /* 收到 $ERR,model_load_fail 后置1, ClearAIError 可清除 */

/**
 * @brief  初始化 OpenMV 串口 (USART3: PC10=TX, PC11=RX)
 */
void OpenMV_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 GPIOC 和 USART3 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PC10 - TX, PC11 - RX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 复用功能配置 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    /* USART 配置 */
    USART_InitStructure.USART_BaudRate            = OMV_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(OMV_USART, &USART_InitStructure);

    /* 配置中断 */
    NVIC_InitStructure.NVIC_IRQChannel                   = OMV_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能接收中断 */
    USART_ITConfig(OMV_USART, USART_IT_RXNE, ENABLE);
    /* 使能 UART */
    USART_Cmd(OMV_USART, ENABLE);

    /* 初始化数据 */
    omv_data.fresh = 0;
    omv_data.tag_id = -1;
}

/**
 * @brief  发送单字节
 */
void OpenMV_SendByte(uint8_t byte)
{
    USART_SendData(OMV_USART, byte);
    while (USART_GetFlagStatus(OMV_USART, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  发送字符串
 */
void OpenMV_SendString(const char *str)
{
    while (*str) {
        OpenMV_SendByte((uint8_t)*str++);
    }
}

/**
 * @brief  向 OpenMV 发送指令
 */
void OpenMV_SendCmd(const char *cmd)
{
    OpenMV_SendString("$CMD,");
    OpenMV_SendString(cmd);
    OpenMV_SendString("\r\n");
}

/**
 * @brief  串口接收中断服务函数
 */
/* 调试: ISR 触发计数 */
volatile uint32_t omv_isr_rx_count = 0;
volatile uint32_t omv_isr_err_count = 0;
/* 调试: 收到的包类型计数 */
volatile uint32_t omv_pkt_hb = 0;
volatile uint32_t omv_pkt_tag = 0;
volatile uint32_t omv_pkt_cls = 0;
volatile uint32_t omv_pkt_qr = 0;
volatile uint32_t omv_pkt_ok = 0;
volatile uint32_t omv_pkt_err = 0;
volatile uint32_t omv_pkt_unk = 0;  /* 诊断: 未识别的 $ 开头行 */

void OMV_USART_IRQHandler(void)
{
    if (USART_GetITStatus(OMV_USART, USART_IT_RXNE) == SET) {
        uint8_t ch;
        uint16_t next;

        /* 帧错误/噪声错误 → 数据不可靠，丢弃 (对标 uart.c 的做法) */
        if (USART_GetFlagStatus(OMV_USART, USART_FLAG_FE) != RESET ||
            USART_GetFlagStatus(OMV_USART, USART_FLAG_NE) != RESET)
        {
            (void)USART_ReceiveData(OMV_USART);
            omv_isr_err_count++;
        }
        else
        {
            ch = (uint8_t)USART_ReceiveData(OMV_USART);
            omv_isr_rx_count++;
            next = (omv_rx_head + 1) % OMV_BUF_SIZE;
            if (next != omv_rx_tail) {
                omv_rx_buf[omv_rx_head] = ch;
                omv_rx_head = next;
            }
        }
    }

    /* 溢出错误清除 */
    if (USART_GetFlagStatus(OMV_USART, USART_FLAG_ORE) == SET) {
        (void)USART_ReceiveData(OMV_USART);
    }
}

/**
 * @brief  从环形缓冲区提取一行 (与 extract_bt_line 逻辑一致)
 */
static int extract_omv_line(char *line, uint16_t max_len)
{
    uint16_t i;
    uint16_t tail;  /* 局部下标: 找到完整行才提交到 omv_rx_tail */
    i = 0;
    tail = omv_rx_tail;
    while (tail != omv_rx_head && i < max_len - 1) {
        char ch;
        ch = (char)omv_rx_buf[tail];
        tail = (tail + 1) % OMV_BUF_SIZE;

        if (ch == '\n' || ch == '\r') {
            line[i] = '\0';
            if (i > 0) {
                omv_rx_tail = tail;  /* 完整行: 提交消费 */
                return 1;
            }
            i = 0;
        } else {
            line[i++] = ch;
        }
    }
    /* 半行不完整: 不更新 omv_rx_tail, 下次继续读 */
    return 0;
}

/**
 * @brief  解析数据行
 *         格式: $TAG,id,cx,cy,distance,angle,width
 *               $QR,payload
 *               $HB
 */
static void parse_omv_line(const char *line)
{
    int tag, cx, cy, dist, angle, width;
    uint16_t i;

    /* $TAG 数据包 */
    if (line[0] == '$' && line[1] == 'T' && line[2] == 'A' && line[3] == 'G')
    {
        if (sscanf(line, "$TAG,%d,%d,%d,%d,%d,%d",
                   &tag, &cx, &cy, &dist, &angle, &width) == 6)
        {
            omv_data.tag_id      = (int16_t)tag;
            omv_data.cx          = (int16_t)cx;
            omv_data.cy          = (int16_t)cy;
            omv_data.distance_cm = (int16_t)dist;
            omv_data.angle_deg   = (int16_t)angle;
            omv_data.pixel_width = (int16_t)width;
            omv_data.timestamp   = g_sys_tick;
            omv_data.fresh       = 1;
            omv_pkt_tag++;

            /* 记录最近标签 (不被消费清除, 供主循环查询) */
            if (tag >= 0)
            {
                omv_last_tag_id   = (int16_t)tag;
                omv_last_tag_tick = g_sys_tick;
            }
        }
        return;
    }

    /* $QR 数据包: $QR,<payload> */
    if (line[0] == '$' && line[1] == 'Q' && line[2] == 'R')
    {
        /* 跳过 "$QR," 前缀 */
        {
            const char *src;
            src = line + 4;  /* 跳过 "$QR," */
            i = 0;
            while (src[i] != '\0' && i < (OMV_QR_TEXT_MAX - 1))
            {
                omv_qr_buf[i] = src[i];
                i++;
            }
            omv_qr_buf[i] = '\0';
            omv_qr_fresh = 1;
            omv_pkt_qr++;
        }
        return;
    }

    /* $CLS 分类数据包: $CLS,class_id,confidence */
    if (line[0] == '$' && line[1] == 'C' && line[2] == 'L' && line[3] == 'S')
    {
        {
            int cls_id, conf;
            if (sscanf(line, "$CLS,%d,%d", &cls_id, &conf) == 2)
            {
                omv_cls_data.class_id   = (int16_t)cls_id;
                omv_cls_data.confidence = (uint8_t)(conf > 100 ? 100 : (conf < 0 ? 0 : conf));
                omv_cls_data.fresh      = 1;
                omv_cls_fresh           = 1;
                omv_pkt_cls++;
            }
        }
        return;
    }

    /* Bug#5: $ERR 错误报文: $ERR,<message> */
    if (line[0] == '$' && line[1] == 'E' && line[2] == 'R' && line[3] == 'R')
    {
        {
            const char *src;
            uint16_t i;
            src = line + 5;  /* 跳过 "$ERR," */
            i = 0;
            while (src[i] != '\0' && i < (sizeof(omv_error_text) - 1))
            {
                omv_error_text[i] = src[i];
                i++;
            }
            omv_error_text[i] = '\0';
            omv_error_fresh = 1;
            omv_pkt_err++;
            /* 模型加载失败 → AI 不可用 (不可逆标志) */
            /* 任何 $ERR 都视为模型异常 (model_load_fail / AI not ready 等) */
            omv_ai_ready = 0;
            omv_model_error = 1;
        }
        return;
    }

    /* $OK,xxx: 任何 $OK 开头的包都计数, $OK,BOOT 额外标记 AI 就绪 */
    if (line[0] == '$' && line[1] == 'O' && line[2] == 'K')
    {
        omv_pkt_ok++;
        if (line[3] == ',' && line[4] == 'B' && line[5] == 'O'
            && !omv_model_error)
        {
            omv_ai_ready = 1;
        }
        return;
    }

    /* $HB 心跳 */
    if (line[0] == '$' && line[1] == 'H' && line[2] == 'B')
    {
        omv_last_hb_tick = g_sys_tick;
        omv_pkt_hb++;
        return;
    }

    /* 诊断: 所有 $ 开头但未被以上任何分支处理的行 */
    if (line[0] == '$')
    {
        omv_pkt_unk++;
    }
}

/**
 * @brief  安全清空环形缓冲区 (短暂关 USART3 RXNE 中断防竞态)
 *         适用于机械臂动作等长时间阻塞后丢弃过时数据
 */
void OpenMV_FlushRx(void)
{
    USART_ITConfig(OMV_USART, USART_IT_RXNE, DISABLE);
    omv_rx_head = 0;
    omv_rx_tail = 0;
    /* 同时清除所有已解析但未消费的脏数据, 防止残留触发误动作 */
    omv_data.fresh    = 0;
    omv_qr_fresh      = 0;
    omv_cls_fresh     = 0;
    omv_error_fresh   = 0;
    omv_last_tag_id   = -1;
    omv_last_tag_tick = 0;
    USART_ITConfig(OMV_USART, USART_IT_RXNE, ENABLE);
}

/**
 * @brief  主循环调用：从环形缓冲区提取一行并解析
 */
void OpenMV_PollLine(void)
{
    char line[128];
    while (extract_omv_line(line, sizeof(line)) == 1) {
        parse_omv_line(line);
    }
}

/**
 * @brief  获取最新视觉数据 (拷贝后清除 fresh 标志)
 *         返回 1=有有效数据, 0=无新数据
 */
int OpenMV_GetData(OpenMV_Data *out)
{
    if (omv_data.fresh == 0 || out == ((OpenMV_Data *)0))
        return 0;
    *out = omv_data;
    omv_data.fresh = 0;
    return 1;
}

/**
 * @brief  查询是否有新数据
 */
int OpenMV_IsFresh(void)
{
    return (omv_data.fresh != 0) ? 1 : 0;
}

/**
 * @brief  查询指定标签是否在最近 max_age ticks 内出现过
 *         不被数据消费影响, ISR 和主循环均可安全调用
 *         tag_id < 0 表示查询"任意标签"
 */
int OpenMV_TagSeenRecently(int16_t tag_id, uint32_t max_age)
{
    uint32_t age;
    if (omv_last_tag_id < 0)
        return 0;
    if (tag_id >= 0 && omv_last_tag_id != tag_id)
        return 0;
    age = g_sys_tick - omv_last_tag_tick;
    return (age <= max_age) ? 1 : 0;
}

/**
 * @brief  获取最近看到的标签 ID 和时间
 */
int16_t OpenMV_GetLastTagId(void)
{
    return omv_last_tag_id;
}

/**
 * @brief  设置视觉工作模式
 */
void OpenMV_SetMode(uint8_t mode)
{
    if (mode == OMV_MODE_APRILTAG)
        OpenMV_SendCmd("MODE,APRILTAG");
    else if (mode == OMV_MODE_COLOR)
        OpenMV_SendCmd("MODE,COLOR");
    else if (mode == OMV_MODE_QRCODE)
        OpenMV_SendCmd("MODE,QRCODE");
    else if (mode == OMV_MODE_AI)
        OpenMV_SendCmd("MODE,AI");
}

/**
 * @brief  设置目标颜色
 */
void OpenMV_SetTargetColor(uint8_t color_id)
{
    char cmd[16];
    sprintf(cmd, "COLOR,%d", color_id);
    OpenMV_SendCmd(cmd);
}

/**
 * @brief  一键切颜色追踪模式 + 目标颜色 (合并命令, 避免两条命令间的竞态)
 */
void OpenMV_SetModeColor(uint8_t color_id)
{
    char cmd[16];
    sprintf(cmd, "MODE,COLOR,%d", color_id);
    OpenMV_SendCmd(cmd);
}
/**
 * @brief  设置目标标签 ID
 */
void OpenMV_SetTargetTag(int16_t tag_id)
{
    char cmd[16];
    sprintf(cmd, "TAG,%d", (int)tag_id);
    OpenMV_SendCmd(cmd);
}

/**
 * @brief  获取 QR 解码文本 (拷贝后清除 fresh)
 */
int OpenMV_GetQRData(OpenMV_QRData *out)
{
    uint16_t i;
    if (omv_qr_fresh == 0 || out == ((OpenMV_QRData *)0))
        return 0;
    i = 0;
    while (omv_qr_buf[i] != '\0' && i < (OMV_QR_TEXT_MAX - 1))
    {
        out->text[i] = omv_qr_buf[i];
        i++;
    }
    out->text[i] = '\0';
    out->fresh = 1;
    omv_qr_fresh = 0;
    return 1;
}

/**
 * @brief  查询 OpenMV 心跳是否存活
 */
int OpenMV_IsAlive(uint32_t max_age)
{
    uint32_t age;
    if (omv_last_hb_tick == 0)
        return 0;
    age = g_sys_tick - omv_last_hb_tick;
    return (age <= max_age) ? 1 : 0;
}

/**
 * @brief  获取 AI 分类结果 (拷贝后清除 fresh)
 */
int OpenMV_GetCLSData(OpenMV_CLSData *out)
{
    if (omv_cls_fresh == 0 || out == ((OpenMV_CLSData *)0))
        return 0;
    out->class_id   = omv_cls_data.class_id;
    out->confidence = omv_cls_data.confidence;
    out->fresh      = 1;
    omv_cls_fresh   = 0;
    return 1;
}

/**
 * @brief  Bug#5: 获取 OpenMV 最近错误信息 (拷贝后清除 fresh)
 *         返回 1=有错误, 0=无错误
 */
int OpenMV_GetLastError(char *err_buf, uint16_t max_len)
{
    uint16_t i;
    if (omv_error_fresh == 0 || err_buf == ((char *)0))
        return 0;
    i = 0;
    while (omv_error_text[i] != '\0' && i < (max_len - 1))
    {
        err_buf[i] = omv_error_text[i];
        i++;
    }
    err_buf[i] = '\0';
    omv_error_fresh = 0;
    return 1;
}

/**
 * @brief  Bug#5: 查询 AI 模型是否就绪
 *         返回 1=AI可用, 0=未就绪 (模型加载失败 / 未收到 BOOT)
 */
int OpenMV_IsAIReady(void)
{
    return (omv_ai_ready != 0) ? 1 : 0;
}

/**
 * @brief  清除 AI 模型错误标志, 允许下次 MODE,AI 重新尝试加载
 *         应在发送 MODE,AI 命令前调用
 */
int OpenMV_IsModelError(void)
{
    return (omv_model_error != 0) ? 1 : 0;
}

void OpenMV_ClearAIError(void)
{
    omv_model_error = 0;
    omv_ai_ready = 0;
}

/**
 * @brief  打印 OpenMV 完整诊断状态
 */
void OpenMV_PrintStatus(void)
{
    uint32_t hb_age;

    if (omv_last_hb_tick == 0) {
        printf("OpenMV: NO heartbeat (never received $HB)\r\n");
    } else {
        hb_age = g_sys_tick - omv_last_hb_tick;
        printf("OpenMV: HB age=%lu ticks (%s)\r\n",
               (unsigned long)hb_age,
               (hb_age <= 200) ? "alive" : "TIMEOUT");
    }

    printf("  AI ready=%d  model_error=%d  last_err=%s\r\n",
           (int)omv_ai_ready, (int)omv_model_error,
           omv_error_fresh ? omv_error_text : "(none)");
    printf("  ISR rx=%lu err=%lu  buf head=%u tail=%u\r\n",
           (unsigned long)omv_isr_rx_count,
           (unsigned long)omv_isr_err_count,
           (unsigned int)omv_rx_head, (unsigned int)omv_rx_tail);
    printf("  PKT: HB=%lu OK=%lu ERR=%lu TAG=%lu CLS=%lu QR=%lu UNK=%lu\r\n",
           (unsigned long)omv_pkt_hb,
           (unsigned long)omv_pkt_ok,
           (unsigned long)omv_pkt_err,
           (unsigned long)omv_pkt_tag,
           (unsigned long)omv_pkt_cls,
           (unsigned long)omv_pkt_qr,
           (unsigned long)omv_pkt_unk);
    /* 直接读 USART3 寄存器 确认外设有没有被正确初始化 */
    printf("  USART3 SR=0x%04X DR=0x%04X BRR=0x%04X CR1=0x%04X\r\n",
           (unsigned int)(OMV_USART->SR  & 0xFFFF),
           (unsigned int)(OMV_USART->DR  & 0xFFFF),
           (unsigned int)(OMV_USART->BRR & 0xFFFF),
           (unsigned int)(OMV_USART->CR1 & 0xFFFF));
    /* PC10/PC11 的 GPIO 模式 */
    {
        uint32_t moder, afr;
        moder = GPIOC->MODER;
        afr   = GPIOC->AFR[1];  /* AFR[1] = pin 8-15 */
        printf("  GPIOC MODER=0x%08X AFR[1]=0x%08X\r\n",
               (unsigned int)moder, (unsigned int)afr);
        printf("  PC10 mode=%u af=%u  PC11 mode=%u af=%u\r\n",
               (unsigned int)((moder >> 20) & 3),
               (unsigned int)((afr >> 8) & 0xF),
               (unsigned int)((moder >> 22) & 3),
               (unsigned int)((afr >> 12) & 0xF));
    }

    /* 显示最近收到的数据类型 */
    {
        OpenMV_Data  omv;
        OpenMV_CLSData cls;
        OpenMV_QRData  qr;

        if (OpenMV_GetData(&omv))
            printf("  TAG: id=%d cx=%d cy=%d dist=%dcm\r\n",
                   (int)omv.tag_id, (int)omv.cx,
                   (int)omv.cy, (int)omv.distance_cm);
        else
            printf("  TAG: no fresh data\r\n");

        if (OpenMV_GetCLSData(&cls))
            printf("  CLS: class=%d conf=%d%%\r\n",
                   (int)cls.class_id, (int)cls.confidence);
        else
            printf("  CLS: no fresh data\r\n");

        if (OpenMV_GetQRData(&qr))
            printf("  QR: %s\r\n", qr.text);
        else
            printf("  QR: no fresh data\r\n");
    }
}
