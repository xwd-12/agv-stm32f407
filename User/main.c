#include "stm32f4xx.h"
#include "./led/bsp_led.h"
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "pwm.h"
#include "pid.h"
#include "line_follow.h"
#include "enconder.h"
#include "motor.h"
#include "line_sensor.h"
#include "smooth_servo.h"
#include "arm_config.h"
#include "command.h"
#include "servo.h"
#include "hc_sr40.h"
#include "uart.h"
#include "action_group.h"
#include "state_machine.h"
#include "commend_openmv.h"
#include "visual_servo.h"
#include "vision_task.h"
#include <string.h>

uint8_t KEYNUM;
uint16_t NUM;
uint16_t i;

#define MODE_MANUAL       0
#define MODE_LINE_FOLLOW  1
#define MODE_POSITION     2
/* MODE_VISUAL_SERVO 3 is defined in visual_servo.h */

volatile uint8_t work_mode = MODE_MANUAL;
volatile uint8_t motor_enable = 0;
Line_follow_Handle line_follow;
PID_Handle pid_positon[4];
int32_t target_position[4] = {0,0,0,0};
PID_Handle pid_motor[4];
volatile int16_t target_speed[4] = {0, 0, 0, 0};
int8_t motor_dir[4] = {1, -1, 1, -1};
int8_t encoder_dir[4] = {-1, -1, -1, -1};
int16_t encoder_max[4] = {2500, 2500, 2500, 0};
VisualServo_Handle visual_servo;
TaskQueue task_queue;
VisionTask_Handle vision_task;
volatile uint8_t qr_trig_flag = 0;   /* 串口调试: 手动触发QR停车 */

// LED快闪n次
static void LedBlink(int n)
{
    int j;
    for (j = 0; j < n; j++) {
        GPIO_ResetBits(GPIOF, GPIO_Pin_9);  // LED ON
        Delay_ms(200);
        GPIO_SetBits(GPIOF, GPIO_Pin_9);    // LED OFF
        Delay_ms(200);
    }
}

// 开机自检：短促驱动每个电机
static void MotorSelfTest(void)
{
    int j;
    for (j = 0; j < 4; j++) {
        Motor_SetSpeed(j, 400);
        Delay_ms(300);
        Motor_SetSpeed(j, 0);
        Delay_ms(200);
        Motor_SetSpeed(j, -400);
        Delay_ms(300);
        Motor_SetSpeed(j, 0);
        Delay_ms(200);
    }
}

int main(void)
{
    int i;
    char cmd[64];

    SystemInit();
    SysTick_Init();  // 启动1ms SysTick定时器，为非阻塞延时提供时间基准
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    DBGMCU->CR &= ~(DBGMCU_CR_TRACE_IOEN | (0x3 << 6));

    Motor_EmergencyBrake();

    LED_Init();
    GPIO_SetBits(GPIOF, GPIO_Pin_9);  // LED off initially

    uart_Init();
    printf("a\r\n");        // 上电自动发送 a
    /* 没接OLED，跳过初始化，避免I2C超时拖慢启动 */
    /* OLED_Init(); */
    /* OLED_Clear(); */
    /* OLED_ShowString(0,0,(u8*)"Init...",16,1); */
    /* BT_Init(); -- 蓝牙未用, USART2空闲, 调试串口改用UART5(PC12/PD2) */
    OpenMV_Init();

    Motor_Init();
    PWM_init();
    Delay_ms(200);          // 等PWM信号稳定后再设舵机角度，避免上电抖动
    Servo_Init();
    smooth_Init();
    ArmSM_Init();
    Action_ParamInit();
    VisionTask_Init(&vision_task);

    Encoder_Init();
    Line_sensor_Init();

    /* 视觉伺服初始化: 任意标签, 15cm 停靠, 速度 150, 默认 PID */
    VisualServo_Init(&visual_servo, -1, 15,
        150, 3.0f, 0.05f, 0.5f, 2.0f, 0.02f, 0.3f, 0.01f);
    TaskQueue_Init(&task_queue);

    lineFollow_Init(&line_follow, 180, 180.0, 2.0, 2.0, 0.01);
    PID_Init(&pid_motor[0], PID_INCREMENTAL, 5.5, 0.16, 0.0, 300, 1000, 300);  // 前轮
    PID_Init(&pid_motor[1], PID_INCREMENTAL, 5.5, 0.16, 0.0, 300, 1000, 300);  // 前轮
    PID_Init(&pid_motor[2], PID_INCREMENTAL, 5.0, 0.14, 0.0, 300, 1000, 300);  // 后轮
    PID_Init(&pid_motor[3], PID_INCREMENTAL, 5.0, 0.14, 0.0, 300, 1000, 300);  // 后轮 开环
    for(i=0; i<4; i++)
        PID_Init(&pid_positon[i], PID_POSITION, 0.5, 0.001, 0, 200, 600, 300);

    Timer_Init();
    PVD_Init();         // 掉电检测：VDD<2.9V时自动刹停全部电机

    // 启动前清零PID和电机输出，避免残留饱和值
    for (i = 0; i < 4; i++) {
        Motor_SetSpeed(i, 0);
        PID_Reset(&pid_motor[i]);
        PID_Reset(&pid_positon[i]);
    }

    motor_enable = 0;
    target_speed[0] = 0;
    target_speed[1] = 0;
    target_speed[2] = 0;
    target_speed[3] = 0;

    // ====== 通电直接进循迹，机械臂后台复位 ======
    work_mode = MODE_LINE_FOLLOW;
    motor_enable = 1;
    Action_Reset();
    while (!Action_ISdle());

    // ====== 开机自检 ======
    OpenMV_SendCmd("MODE,QRCODE");  /* 启动巡线QR码扫描模式 */
    LedBlink(3);        // LED快闪3次 = MCU运行正常

    // 排空上电期间 USB-TTL 噪点（ISR 可能收了乱码字节）
    while (uart_rb_getchar() >= 0);

    // 记录主循环启动时刻（用于抑制启动噪声）
    {
        uint32_t loop_start_tick;
        loop_start_tick = g_sys_tick;

    while (1)
    {
        /* 轮询收数+逐字符回显+命令行缓冲 */
        {
            static uint8_t  line_idx = 0;
            static char     line_buf[64];
            int ch_int;
            uint8_t ch;
            uint32_t uptime;

            uptime = g_sys_tick - loop_start_tick;

            /* 先排空环形缓冲区(ISR在printf期间静默收集的字符) */
            while ((ch_int = uart_rb_getchar()) >= 0)
            {
                ch = (uint8_t)ch_int;
                /* 启动后3秒内静默, 只记录命令行不显 */
                if (uptime < 300)
                {
                    if (ch == '\r' || ch == '\n')
                    {
                        if (line_idx > 0)
                        {
                            line_buf[line_idx] = '\0';
                            line_idx = 0;
                            uart_set_bt_output(0);
                            Commend_Parse(line_buf);
                        }
                    }
                    else if (line_idx < (sizeof(line_buf) - 1))
                    {
                        line_buf[line_idx++] = ch;
                    }
                    continue;
                }
                /* 只回显字母数字+常用符号+空白, 滤掉$TAG等OpenMV数据 */
                if ((ch >= 'a' && ch <= 'z') ||
                    (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') ||
                    ch == ' ' || ch == '_' || ch == '-' ||
                    ch == '.' || ch == ':' || ch == '=' ||
                    ch == '\r' || ch == '\n')
                    uart_sendbyte(ch);
                if (ch == '\r' || ch == '\n')
                {
                    if (line_idx > 0)
                    {
                        line_buf[line_idx] = '\0';
                        line_idx = 0;
                        uart_set_bt_output(0);
                        Commend_Parse(line_buf);
                    }
                }
                else if (line_idx < (sizeof(line_buf) - 1))
                {
                    line_buf[line_idx++] = ch;
                }
            }

            /* RX 完全走 ISR → 环形缓冲区, 主循环只从缓冲区取
             * (直接轮询 RXNE 会和 ISR 抢同一个硬件寄存器导致掉字) */
        }

        // ====== 定时自动任务 (已注释, 改用 QR 触发) ======
#if 0
        {
            static uint8_t  auto_phase = 0;  /* 0=首圈抓, 1=二圈放, 2=完成 */
            static uint32_t phase1_start_tick = 0;  /* 抓取完成时刻 */
            switch (auto_phase) {
                case 0:  /* 第一圈: 跑5s → 抓取 */
                    if (g_sys_tick >= 500) {
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            PID_Reset(&pid_motor[i]);
                            Motor_SetSpeed(i, 0);
                        }
                        Delay_ms(300);
                        Action_Grasp();
                        phase1_start_tick = g_sys_tick;
                        auto_phase = 1;
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                    }
                    break;
                case 1:  /* 抓完再跑17s → 放置 */
                    if (g_sys_tick - phase1_start_tick >= 1700) {
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            PID_Reset(&pid_motor[i]);
                            Motor_SetSpeed(i, 0);
                        }
                        Delay_ms(300);
                        Action_Place(act_angle[ACT_PLACE_WAIST]);
                        auto_phase = 2;
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                    }
                    break;
                default:  /* 完成, 之后纯巡线 */
                    break;
            }
        }
#endif

        /* 先解析 OpenMV 最新数据, 确保 AI 管线拿到的是最新帧 */
        OpenMV_PollLine();

        // ====== AI 自动识别抓取: 冷却→停车→AI分类→颜色对准→执行 ======
        // 注意: 当前跳过QR, 直接周期触发AI识别 (二维码暂时没弄好)

        /* 对准目标距离 — 机械臂短就改小 */
        #define GRASP_TARGET_CM    6    /* 抓取目标距离 (cm) */
        #define PLACE_TARGET_CM    6    /* 放置目标距离 (cm) */
        #define ALIGN_DIST_DEAD    3    /* 距离对准死区 (±cm) */
        /* 摄像头-机械臂横向偏移校准: 画面中夹爪正对物体时的 cx 值
           默认160=画面正中, 夹偏左则改大(如180), 偏右则改小(如140) */
        #define GRASP_CX_TARGET   190   /* 夹子偏右, 继续加大 */
        #define MISSION_COUNT      3   /* 依次夹取物体数 */

        {
            /* ---- 多物体依次夹取: 红(0)→黄(2)→绿(1) ---- */
            static int8_t  mission_class[MISSION_COUNT] = {0, 2, 1};
            /* 按颜色(1=红 2=绿 3=黄)对准参数, 索引=color-1 */
            static int16_t color_cx[3]        = {190, 190, 190};
            static int16_t color_grasp_dist[3] = {  6,   6,   6};  /* 红 绿 黄 */
            static uint8_t mission_idx  = 2;  /* TODO: 调试完改回0 */
            static uint8_t mission_done = 0;
            static uint32_t mission_last_print = 0;
            static uint8_t  qr_phase = 0;      /* 0=冷却等待 1=等AI 2=对准 3=执行 */
            static uint32_t qr_phase_tick = 0;
            static uint32_t qr_cooldown_end = 0; /* 冷却结束时刻, 防同站点重复触发 */
            static uint8_t  qr_is_place = 0;   /* 0=抓 1=放 */
            static uint8_t  qr_target_color = 1; /* AI分类映射的颜色 */
            static uint8_t  qr_aligned_cnt = 0;  /* 对准连续计数 */
            static uint8_t  qr_align_phase = 0; /* 0=腰座横向对准 1=车身纵向调整 */
            static uint8_t  qr_scan_started = 0; /* 0=未启动腰座左扫 1=已启动 */
            static float    qr_waist_center = 50.0f; /* AI检测时的腰座角度, 扫描中心 */
            /* Bug#1: Phase 2 硬超时计时 (不受数据帧刷新影响) */
            static uint32_t qr_phase2_start_tick = 0;
            static uint8_t  qr_phase2_arm_done = 0; /* Phase2.0 摆臂到位标志 */
            /* Bug#2: 扫描状态移至外层, 跨任务可复位 */
            static uint16_t nodata_start = 0;
            static uint8_t  scan_step   = 0;
            static uint16_t scan_tick   = 0;
            static uint16_t dbg_tick    = 0;  /* printf 节流 */
            /* Bug#4: Phase 1 主动扫描状态 */
            static uint8_t  phase1_scan_step = 0;
            static uint16_t phase1_scan_tick = 0;
            /* Phase 4 找圈放置整体超时 */
            static uint32_t qr_place_phase_start = 0;
            OpenMV_CLSData  cls;
            OpenMV_Data     omv;
            int j;

            switch (qr_phase) {
            case 0:  /* 巡线中, 等待QR码 → 减速靠近 → 15cm停车 → 进AI */
                {
                    OpenMV_QRData qr;
                    OpenMV_Data     omv;
                    uint32_t        app_start;
                    int             app_done;

                    /* 任务已完成: 停车, 忽略所有QR */
                    if (mission_done) {
                        if (motor_enable) {
                            motor_enable = 0;
                            work_mode = MODE_MANUAL;
                            for (j = 0; j < 4; j++) {
                                target_speed[j] = 0;
                                PID_Reset(&pid_motor[j]);
                                Motor_SetSpeed(j, 0);
                            }
                        }
                        if (g_sys_tick - mission_last_print > 1000) {
                            mission_last_print = g_sys_tick;
                            printf("Mission complete. %d objects done.\r\n",
                                   MISSION_COUNT);
                        }
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }
                    /* 上电后先巡线5秒稳定 */
                    if (g_sys_tick < 500)
                        break;
                    /* 冷却期内丢弃所有QR数据, 防止残留触发 */
                    if (g_sys_tick < qr_cooldown_end) {
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }
                    /* 检测QR码 */
                    if (!OpenMV_GetQRData(&qr) && !qr_trig_flag)
                        break;
                    if (qr_trig_flag) {
                        printf("QR: manual trigger\r\n");
                        qr_trig_flag = 0;
                    } else {
                        printf("QR: %s -> approach\r\n", qr.text);
                    }
                    if (ArmSM_IsBusy()) {
                        printf("QR: arm busy, skip\r\n");
                        qr_cooldown_end = g_sys_tick + 500;
                        break;
                    }
                    /* 切手动, 用QR位置数据慢速靠近 */
                    work_mode = MODE_MANUAL;
                    for (j = 0; j < 4; j++)
                        target_speed[j] = 0;
                    motor_enable = 1;
                    app_done = 0;
                    app_start = g_sys_tick;
                    {
                        uint8_t lost_cnt;
                        lost_cnt = 0;
                        while (!app_done && g_sys_tick - app_start < 600) {
                            OpenMV_PollLine();
                            if (OpenMV_GetData(&omv)
                                && omv.tag_id == 0
                                && omv.distance_cm > 0) {
                                lost_cnt = 0;
                                if (omv.distance_cm <= 15) {
                                    app_done = 1;
                                } else {
                                    for (j = 0; j < 4; j++)
                                        target_speed[j] = 80;
                                }
                            } else {
                                lost_cnt++;
                                for (j = 0; j < 4; j++)
                                    target_speed[j] = 0;
                                /* 连续丢5帧=靠太近出画幅, 也算到位 */
                                if (lost_cnt >= 5) {
                                    printf("QR: lost (too close?)\r\n");
                                    app_done = 1;
                                }
                            }
                            Delay_ms(100);
                        }
                    }
                    /* 立即制动 */
                    motor_enable = 0;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        PID_Reset(&pid_motor[j]);
                        Motor_SetSpeed(j, 0);
                    }
                    printf("QR: stop at %d cm\r\n", (int)omv.distance_cm);
                    Delay_ms(300);
                    /* 摆臂 → 切AI */
                    smooth_Settarget(1, 80, 600);
                    smooth_Settarget(2, 60, 600);
                    smooth_Settarget(0, 50, 600);
                    OpenMV_ClearAIError();
                    OpenMV_SendCmd("MODE,AI");
                    qr_phase = 1;
                    qr_phase_tick = g_sys_tick;
                }
                break;

            case 1:  /* 等AI模型加载 → 腰座扫描 */
                OpenMV_PollLine();

                /* 模型加载等待 (仅在扫描开始前) */
                if (!qr_scan_started) {
                    if (OpenMV_IsModelError()) {
                        printf("AI: model load failed, abort\r\n");
                        OpenMV_SendCmd("MODE,QRCODE");
                        motor_enable = 0;
                        for (j = 0; j < 4; j++) {
                            target_speed[j] = 0;
                            Motor_SetSpeed(j, 0);
                        }
                        qr_cooldown_end = g_sys_tick + 3000;
                        qr_phase = 0;
                        break;
                    }
                    if (g_sys_tick - qr_phase_tick > 300
                        && !OpenMV_IsAlive(1200))
                    {
                        printf("AI: OpenMV crash (no HB), abort\r\n");
                        OpenMV_SendCmd("MODE,QRCODE");
                        motor_enable = 0;
                        for (j = 0; j < 4; j++) {
                            target_speed[j] = 0;
                            Motor_SetSpeed(j, 0);
                        }
                        qr_cooldown_end = g_sys_tick + 3000;
                        qr_phase = 0;
                        break;
                    }
                    if (!OpenMV_IsAIReady() && g_sys_tick - qr_phase_tick < 6000) {
                        break;
                    }
                    if (!OpenMV_IsAIReady()) {
                        printf("AI: wait timeout 60s, abort\r\n");
                        OpenMV_SendCmd("MODE,QRCODE");
                        motor_enable = 0;
                        for (j = 0; j < 4; j++) {
                            target_speed[j] = 0;
                            Motor_SetSpeed(j, 0);
                        }
                        qr_cooldown_end = g_sys_tick + 3000;
                        qr_phase = 0;
                        break;
                    }
                }
                /* 首次进入: 记录当前腰座角度作为扫描中心 (与放置找圈逻辑一致) */
                if (!qr_scan_started) {
                    qr_waist_center = smooth_GetCurrentAngle(0);
                    qr_scan_started = 1;
                    phase1_scan_step = 0;   /* 扫描步序 0-5 */
                    phase1_scan_tick = (uint16_t)g_sys_tick;
                }
                /* 主动扫描: 每步±5°, 3步换向, 到位后再识别 */
                {
                    float scan_target;
                    int   scan_dir, scan_cnt;

                    if (!smoothservo_IsBusy(0)
                        && (uint16_t)(g_sys_tick - phase1_scan_tick) > 80) {
                        phase1_scan_step = (phase1_scan_step + 1) % 6;
                        phase1_scan_tick = (uint16_t)g_sys_tick;

                        scan_dir = (phase1_scan_step < 3) ? 1 : -1;
                        scan_cnt = phase1_scan_step % 3;
                        scan_target = qr_waist_center +
                            (float)(scan_dir * scan_cnt * 5);

                        if (scan_target < 0.0f)   scan_target = 0.0f;
                        if (scan_target > 180.0f) scan_target = 180.0f;
                        printf("Scan: waist->%.0f dir=%d cnt=%d\r\n",
                               (double)scan_target, scan_dir, scan_cnt);
                        smooth_Settarget(0, (uint16_t)scan_target, 600);
                    }
                }
                /* AI识别: 置信度≥25% (红黄绿通用, 黄色偏低放宽) */
                if (OpenMV_GetCLSData(&cls)
                    && cls.class_id == mission_class[mission_idx]
                    && cls.confidence >= 25) {
                    printf("AI: class=%d conf=%d%% -> step[%d] match\r\n",
                           (int)cls.class_id, (int)cls.confidence,
                           (int)mission_idx);
                    qr_scan_started = 0;
                    qr_waist_center = smooth_GetCurrentAngle(0);
                    smooth_Settarget(0, (uint16_t)qr_waist_center, 300);
                    qr_target_color = (uint8_t)(cls.class_id + 1);
                    if (qr_target_color < 1) qr_target_color = 1;
                    if (qr_target_color > 3) qr_target_color = 3;
                    /* 切 OpenMV 到颜色追踪模式 */
                    {
                        char cbuf[16];
                        sprintf(cbuf, "MODE,COLOR,%d",
                                (int)qr_target_color);
                        OpenMV_SendCmd(cbuf);
                    }
                    /* 等 COLOR 首帧 */
                    {
                        OpenMV_Data dummy;
                        uint32_t wait_start;
                        int      retry;
                        OpenMV_PollLine();
                        (void)OpenMV_GetData(&dummy);
                        for (retry = 0; retry < 2; retry++) {
                            wait_start = g_sys_tick;
                            while (g_sys_tick - wait_start < 300) {
                                OpenMV_PollLine();
                                if (OpenMV_IsFresh()) break;
                            }
                            if (OpenMV_IsFresh()) break;
                            {
                                char cbuf[16];
                                sprintf(cbuf, "MODE,COLOR,%d",
                                        (int)qr_target_color);
                                OpenMV_SendCmd(cbuf);
                            }
                            printf("Phase2: retry COLOR mode\r\n");
                        }
                        if (!OpenMV_IsFresh())
                            printf("Phase2: WARN no COLOR data"
                                   " after retry\r\n");
                    }
                    qr_aligned_cnt = 0;
                    qr_align_phase = 0;
                    motor_enable = 1;
                    work_mode = MODE_MANUAL;
                    nodata_start = 0;
                    scan_step    = 2;
                    scan_tick    = (uint16_t)g_sys_tick;
                    dbg_tick     = 0;
                    qr_phase = 2;
                    qr_phase_tick = g_sys_tick;
                    qr_phase2_start_tick = g_sys_tick;
                    qr_phase2_arm_done = 0;
                    printf("Phase2: enter, target_color=%d\r\n",
                           (int)qr_target_color);
                }
                else if (g_sys_tick - qr_phase_tick > 2000) {
                    printf("AI: no target[%d] (class=%d) detected, abort\r\n",
                           (int)mission_idx,
                           (int)mission_class[mission_idx]);
                    qr_scan_started = 0;
                    OpenMV_SendCmd("MODE,QRCODE");
                    qr_cooldown_end = g_sys_tick + 1500;
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    qr_phase = 0;
                }
                break;

            case 2:  /* 对准: 腰座横向→车身纵向, 无数据时自动扫描 */
                {
                    /* 确保拿到最新一帧, 避免数据被底部 OpenMV_PollLine 抢先消费 */
                    OpenMV_PollLine();

                    if (OpenMV_GetData(&omv)) {
                        int16_t error_x, error_d, abs_x, abs_d;
                        int16_t spd;

                        /* 有数据了, 复位扫描状态, 更新扫描中心 */
                        nodata_start = 0;
                        scan_step    = 2;  /* 从中心开始, 不跳反方向 */
                        scan_tick    = (uint16_t)g_sys_tick;
                        qr_waist_center = smooth_GetCurrentAngle(0);

                        error_x = omv.cx - color_cx[qr_target_color - 1];
                        error_d = omv.distance_cm - color_grasp_dist[qr_target_color - 1];
                        if (error_x < 0) abs_x = -error_x; else abs_x = error_x;
                        if (error_d < 0) abs_d = -error_d; else abs_d = error_d;

                        if (qr_align_phase == 0) {
                            /* ---- 子阶段0: 腰座横向对准 ---- */
                            int16_t phase0_threshold;
                            for (j = 0; j < 4; j++)
                                target_speed[j] = 0;

                            /* 腰座比例旋转: 误差(px) × 0.04°/px */
                            {
                                float cur_angle, delta, new_angle;
                                cur_angle = smooth_GetCurrentAngle(0);
                                delta = -(float)error_x * 0.04f;
                                if (delta > 5.0f)  delta = 5.0f;
                                if (delta < -5.0f) delta = -5.0f;
                                new_angle = cur_angle + delta;
                                if (new_angle < 0.0f)   new_angle = 0.0f;
                                if (new_angle > 180.0f) new_angle = 180.0f;
                                smooth_Settarget(0, (uint16_t)new_angle, 100);
                            }

                            if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                                printf("Phase2.0: cx=%d dist=%d waist=%.0f\r\n",
                                       (int)omv.cx, (int)omv.distance_cm,
                                       (double)smooth_GetCurrentAngle(0));
                                dbg_tick = (uint16_t)g_sys_tick;
                            }

                            /* 放宽到±25px适应黄绿色块噪声，通过后Phase2.1精调 */
                            phase0_threshold = 25;
                            if (abs_x <= phase0_threshold) {
                                qr_aligned_cnt++;
                                if (qr_aligned_cnt >= 3) {
                                    printf("Phase2.0 done: cx aligned\r\n");
                                    smooth_Settarget(3, act_angle[ACT_GRIPPER_OPEN], 500);
                                    qr_align_phase = 1;
                                    qr_aligned_cnt = 0;
                                }
                            } else {
                                qr_aligned_cnt = 0;
                            }
                        } else {
                            /* ---- 子阶段1: 腰座精调 + 车身前后 ---- */
                            /* 腰座精调 */
                            {
                                float cur_angle, delta, new_angle;
                                cur_angle = smooth_GetCurrentAngle(0);
                                delta = -(float)error_x * 0.04f;
                                if (delta > 5.0f)  delta = 5.0f;
                                if (delta < -5.0f) delta = -5.0f;
                                new_angle = cur_angle + delta;
                                if (new_angle < 0.0f)   new_angle = 0.0f;
                                if (new_angle > 180.0f) new_angle = 180.0f;
                                smooth_Settarget(0, (uint16_t)new_angle, 100);
                            }

                            /* 车身分级前后移动: 距离越远速度越快
                               NOTE: 最低 200, 增量式PID在低速堵转时Kp项=0, 低于200无法克服静摩擦 */
                            spd = 0;
                            {
                                int16_t ad;
                                ad = ALIGN_DIST_DEAD;
                                if      (error_d > 10) spd = 350;  /* 很远, 快速前进 */
                                else if (error_d >  5) spd = 250;
                                else if (error_d > ad) spd = 200;  /* 微调前进 */
                                else if (error_d < -10) spd = -350; /* 太近, 快速后退 */
                                else if (error_d <  -5) spd = -250;
                                else if (error_d < -ad) spd = -200; /* 微调后退 */
                            }
                            for (j = 0; j < 4; j++)
                                target_speed[j] = spd;

                            if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                                printf("Phase2.1: cx=%d dist=%d err_d=%d spd=%d\r\n",
                                       (int)omv.cx, (int)omv.distance_cm,
                                       (int)error_d, (int)spd);
                                dbg_tick = (uint16_t)g_sys_tick;
                            }

                            if (abs_x <= 10 && (abs_d <= ALIGN_DIST_DEAD || omv.distance_cm <= 4)) {
                                qr_aligned_cnt++;
                                if (qr_aligned_cnt >= 3) {
                                    motor_enable = 0;
                                    for (j = 0; j < 4; j++) {
                                        target_speed[j] = 0;
                                        PID_Reset(&pid_motor[j]);
                                        Motor_SetSpeed(j, 0);
                                    }
                                    printf("Aligned: cx=%d dist=%d\r\n",
                                           (int)omv.cx, (int)omv.distance_cm);
                                    /* 复位扫描状态, 防止残留到下次任务 */
                                    nodata_start = 0;
                                    scan_step    = 0;
                                    scan_tick    = 0;
                                    qr_phase = 3;
                                    qr_phase_tick = g_sys_tick;
                                }
                            } else {
                                qr_aligned_cnt = 0;
                            }
                        }

                        qr_phase_tick = g_sys_tick;
                    }
                    else
                    {
                        float scan_target;
                        int16_t scan_dir_local;
                        /* 丢失目标 → 立即停车 */
                        for (j = 0; j < 4; j++) {
                            target_speed[j] = 0;
                            Motor_SetSpeed(j, 0);
                        }
                        /* 不重置对齐计数: 颜色色块偶有丢帧, 保留累积进度 */

                        /* 记录首次无数据时刻 */
                        if (nodata_start == 0)
                            nodata_start = (uint16_t)g_sys_tick;

                        /* 丢数据等0.5s再扫 (跟放置一样快速响应) */
                        if ((uint16_t)(g_sys_tick - nodata_start) <= 50)
                            break;

                        /* 跟放置一样的扫描逻辑: 左右交替扫, 每步5°, 逐步扩大 */
                        scan_dir_local = 1;
                        if ((scan_step / 3) & 1)
                            scan_dir_local = -1;
                        {
                            int16_t step_in_dir;
                            step_in_dir = scan_step % 3;
                            scan_target = qr_waist_center +
                                (float)(scan_dir_local * (step_in_dir + 1) * 5);
                        }
                        if (scan_target < 0.0f)   scan_target = 0.0f;
                        if (scan_target > 180.0f) scan_target = 180.0f;
                        smooth_Settarget(0, (uint16_t)scan_target, 600);

                        /* 每150 ticks换一步, 等腰座到位再切COLOR */
                        if ((uint16_t)(g_sys_tick - scan_tick) > 150) {
                            scan_step = (scan_step + 1) % 6;
                            scan_tick = (uint16_t)g_sys_tick;
                            /* 重新发 COLOR 命令防丢 */
                            {
                                char cbuf[16];
                                sprintf(cbuf, "MODE,COLOR,%d",
                                        (int)qr_target_color);
                                OpenMV_SendCmd(cbuf);
                            }
                        }

                        if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                            printf("Phase2: scan waist=%.0f step=%d center=%.0f\r\n",
                                   (double)scan_target, (int)scan_step,
                                   (double)qr_waist_center);
                            dbg_tick = (uint16_t)g_sys_tick;
                        }
                    }
                }
                /* Bug#1: 10s 硬超时 (从进入 Phase 2 算起, 不受数据帧刷新影响) */
                if (g_sys_tick - qr_phase2_start_tick > 1000) {
                    printf("Align timeout (10s), abort\r\n");
                    motor_enable = 0;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        Motor_SetSpeed(j, 0);
                    }
                    OpenMV_SendCmd("MODE,QRCODE");
                    qr_cooldown_end = g_sys_tick + 1500;
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    qr_phase = 0;
                    /* Bug#2: 复位扫描状态, 防止残留到下次任务 */
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                }
                break;

            case 3:  /* 执行动作: 保持腰座对准角度抓取 */
                motor_enable = 0;
                for (j = 0; j < 4; j++) {
                    target_speed[j] = 0;
                    Motor_SetSpeed(j, 0);
                }
                Delay_ms(200);
                if (qr_is_place) {
                    printf("Action: PLACE\r\n");
                    if (ArmSM_RequestPlace(110)) {
                        Action_Place(act_angle[ACT_PLACE_WAIST]);
                        ArmSM_NotifyComplete();
                    } else {
                        printf("Action: arm busy, skip place\r\n");
                    }
                } else {
                    uint16_t saved_waist;
                    printf("Action: GRASP (waist=%d)\r\n",
                           (int)smooth_GetCurrentAngle(0));
                    /* Bug#3: 检查 ArmSM 返回值, 臂忙时跳过抓取 */
                    if (ArmSM_RequestGrasp()) {
                        /* 临时替换腰座角度为对准值, 避免 Action_Grasp 覆盖 */
                        saved_waist = act_angle[ACT_APPROACH_WAIST];
                        act_angle[ACT_APPROACH_WAIST] =
                            (uint16_t)smooth_GetCurrentAngle(0);
                        Action_Grasp();
                        act_angle[ACT_APPROACH_WAIST] = saved_waist;
                        ArmSM_NotifyComplete();
                    } else {
                        printf("Action: arm busy, skip grasp\r\n");
                    }
                }
                /* 清空机械臂阻塞期间积压的脏数据, 避免误解析 */
                OpenMV_FlushRx();
                if (qr_is_place) {
                    /* 放置完成 → 回 Phase 0 冷却 */
                    OpenMV_SendCmd("MODE,QRCODE");
                    qr_cooldown_end = g_sys_tick + 1500;
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    /* 起步预加载 */
                    target_speed[0] = 180;
                    target_speed[1] = 180;
                    target_speed[2] = 180;
                    target_speed[3] = 170;
                    qr_is_place = 0;    /* 重置，下次为抓取 */
                    /* 推进任务序列 */
                    mission_idx++;
                    if (mission_idx >= MISSION_COUNT) {
                        mission_done = 1;
                        printf("ALL DONE: %d objects placed.\r\n",
                               MISSION_COUNT);
                    } else {
                        printf("Mission->%d: next class_id=%d\r\n",
                               (int)mission_idx,
                               (int)mission_class[mission_idx]);
                    }
                    qr_phase = 0;
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                    dbg_tick     = 0;
                    printf("Place done, resume line follow\r\n");
                } else {
                    /* 抓取完成 → 进入放置流水线 */
                    OpenMV_SendCmd("MODE,QRCODE");
                    qr_cooldown_end = g_sys_tick + 500;  /* 5s冷却，驶离QR码 */
                    qr_place_phase_start = g_sys_tick;   /* 启动放置超时计时 */
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    /* 起步预加载: 避免循迹PID从0爬坡 */
                    target_speed[0] = 180;
                    target_speed[1] = 180;
                    target_speed[2] = 180;
                    target_speed[3] = 170;  /* 右后电机无编码器, 补偿-10 */
                    qr_is_place = 1;    /* 下次执行动作为放置 */
                    qr_phase = 4;       /* 进入放置流水线 */
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                    dbg_tick     = 0;
                    printf("Grasp done, enter place pipeline\r\n");
                }
                break;

            case 4:  /* 巡线+QR扫箱子: 扫到QR→停车→切COLOR看圈颜色 */
                {
                    OpenMV_QRData qr;
                    OpenMV_PollLine();

                    /* 冷却期内丢弃所有QR数据, 防止残留触发假停车 */
                    if (g_sys_tick < qr_cooldown_end) {
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }

                    if (OpenMV_GetQRData(&qr)) {
                        int color_found = 0;  /* 局部变量, 不复用qr_scan_started */
                        /* 扫到二维码 → 停车 → 摆臂 → 切 COLOR 看箱子上的圈 */
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        {
                            int jj;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                        }
                        Delay_ms(300);

                        /* 先摆臂到识别位: 腰座转+大臂小臂下探, 让摄像头对准箱子色圈 */
                        smooth_Settarget(0, act_angle[ACT_APPROACH_WAIST], 1000);
                        smooth_Settarget(1, act_angle[ACT_PLACE_SHOULDER], 1000);
                        smooth_Settarget(2, act_angle[ACT_APPROACH_ELBOW], 1200);
                        /* 等腰座+大臂到位再扫描 (1000ms走完+余量) */
                        Delay_ms(1500);

                        /* 腰座先转动到位, 再切 COLOR 识别: 每步停留1s */
                        {
                            float scan_center, scan_angle;
                            int   scan_dir, scan_step_cnt;
                            uint32_t scan_deadline, scan_next_tick;

                            color_found = 0;
                            scan_center = smooth_GetCurrentAngle(0);
                            scan_dir    = 1;       /* 1=右扫, -1=左扫 */
                            scan_step_cnt = 0;
                            scan_deadline  = g_sys_tick + 1500; /* 总超时 15s */
                            scan_next_tick = g_sys_tick;

                            OpenMV_PollLine();
                            (void)OpenMV_GetData(&omv);

                            while (g_sys_tick < scan_deadline && !color_found) {
                                if (g_sys_tick >= scan_next_tick) {
                                    scan_next_tick = g_sys_tick + 150;  /* 700ms转动+400ms识别=1.1s, 留余量 */
                                    scan_step_cnt++;
                                    if (scan_step_cnt >= 3) {
                                        scan_step_cnt = 0;
                                        scan_dir = -scan_dir;
                                    }
                                    scan_angle = scan_center +
                                        (float)(scan_dir * scan_step_cnt * 5);
                                    if (scan_angle < 0.0f)   scan_angle = 0.0f;
                                    if (scan_angle > 180.0f) scan_angle = 180.0f;
                                    printf("Place: rotate waist to %.0f\r\n",
                                           (double)scan_angle);
                                    /* 1.先转动腰座到位 */
                                    smooth_Settarget(0, (uint16_t)scan_angle, 600);
                                    /* 等腰座真正到位再识别 (轮询,最长等2s) */
                                    {
                                        uint32_t wt = g_sys_tick;
                                        while (smoothservo_IsBusy(0) && g_sys_tick - wt < 200) {
                                            Delay_ms(10);
                                        }
                                    }
                                    printf("Place: waist at %.0f\r\n",
                                           (double)smooth_GetCurrentAngle(0));

                                    /* 2.再切 COLOR 模式识别 */
                                    {
                                        char cbuf[16];
                                        sprintf(cbuf, "MODE,COLOR,%d", (int)qr_target_color);
                                        OpenMV_SendCmd(cbuf);
                                    }
                                    /* 等几帧 COLOR 数据 */
                                    {
                                        uint32_t t0 = g_sys_tick;
                                        while (g_sys_tick - t0 < 40) {
                                            OpenMV_PollLine();
                                            if (OpenMV_GetData(&omv)
                                                && omv.tag_id == (int16_t)qr_target_color) {
                                                printf("Place: color=%d d=%d w=%d waist=%.0f\r\n",
                                                       (int)qr_target_color,
                                                       (int)omv.distance_cm,
                                                       (int)omv.pixel_width,
                                                       (double)smooth_GetCurrentAngle(0));
                                                /* 只过滤明显太远的 (>50cm) 或太小的 (<5px) */
                                                if (omv.distance_cm <= 50
                                                    && omv.pixel_width >= 5) {
                                                    printf("Place: color=%d accepted\r\n",
                                                           (int)qr_target_color);
                                                    color_found = 1;
                                                    break;
                                                } else {
                                                    printf("Place: color=%d rejected (too far/small)\r\n",
                                                           (int)qr_target_color);
                                                }
                                            }
                                            Delay_ms(10);
                                        }
                                    }
                                }
                                Delay_ms(20);
                            }
                        }
                        /* 判断颜色是否匹配 */
                        if (color_found) {
                            printf("Place: color=%d match! aligning\r\n",
                                   (int)qr_target_color);
                            qr_waist_center = smooth_GetCurrentAngle(0);
                            qr_aligned_cnt = 0;
                            qr_align_phase = 0;
                            nodata_start = 0;
                            scan_step    = 2;
                            scan_tick    = (uint16_t)g_sys_tick;
                            dbg_tick     = 0;
                            motor_enable = 1;
                            qr_phase = 5;
                            qr_phase_tick = g_sys_tick;
                            qr_phase2_start_tick = g_sys_tick;
                        } else {
                            /* 没收到颜色数据, 可能是箱子无色圈或光线差, 直接强制放置 */
                            printf("Place: no color data, force place\r\n");
                            qr_phase = 6;
                            qr_phase_tick = g_sys_tick;
                        }
                    }

                    /* 60s 超时找不到匹配箱子 → 原地放置 */
                    if (g_sys_tick - qr_place_phase_start > 6000) {
                        printf("Place: no matching box (60s), force place\r\n");
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        {
                            int jj;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                        }
                        qr_phase = 6;
                        qr_phase_tick = g_sys_tick;
                    }
                }
                break;

            case 5:  /* 对准箱子上的圈: 腰座横向→车身前后 (同Phase2) */
                {
                    OpenMV_PollLine();

                    if (OpenMV_GetData(&omv)) {
                        int16_t error_x, error_d, abs_x, abs_d;
                        int16_t spd;
                        int16_t phase0_threshold;

                        nodata_start = 0;
                        scan_step    = 2;
                        scan_tick    = (uint16_t)g_sys_tick;
                        qr_waist_center = smooth_GetCurrentAngle(0);

                        error_x = omv.cx - color_cx[qr_target_color - 1];
                        error_d = omv.distance_cm - (int16_t)PLACE_TARGET_CM;
                        if (error_x < 0) abs_x = -error_x; else abs_x = error_x;
                        if (error_d < 0) abs_d = -error_d; else abs_d = error_d;

                        if (qr_align_phase == 0) {
                            /* 子阶段0: 腰座横向对准 */
                            {
                                int jj;
                                for (jj = 0; jj < 4; jj++)
                                    target_speed[jj] = 0;
                            }
                            {
                                float cur_angle, delta, new_angle;
                                cur_angle = smooth_GetCurrentAngle(0);
                                delta = -(float)error_x * 0.04f;
                                if (delta > 5.0f)  delta = 5.0f;
                                if (delta < -5.0f) delta = -5.0f;
                                new_angle = cur_angle + delta;
                                if (new_angle < 0.0f)   new_angle = 0.0f;
                                if (new_angle > 180.0f) new_angle = 180.0f;
                                smooth_Settarget(0, (uint16_t)new_angle, 100);
                            }
                            if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                                printf("Place5.0: cx=%d dist=%d\r\n",
                                       (int)omv.cx, (int)omv.distance_cm);
                                dbg_tick = (uint16_t)g_sys_tick;
                            }
                            phase0_threshold = 25;
                            if (abs_x <= phase0_threshold) {
                                qr_aligned_cnt++;
                                if (qr_aligned_cnt >= 3) {
                                    printf("Place5.0 done\r\n");
                                    qr_align_phase = 1;
                                    qr_aligned_cnt = 0;
                                }
                            } else {
                                qr_aligned_cnt = 0;
                            }
                        } else {
                            /* 子阶段1: 腰座精调 + 车身前后 */
                            {
                                float cur_angle, delta, new_angle;
                                cur_angle = smooth_GetCurrentAngle(0);
                                delta = -(float)error_x * 0.04f;
                                if (delta > 5.0f)  delta = 5.0f;
                                if (delta < -5.0f) delta = -5.0f;
                                new_angle = cur_angle + delta;
                                if (new_angle < 0.0f)   new_angle = 0.0f;
                                if (new_angle > 180.0f) new_angle = 180.0f;
                                smooth_Settarget(0, (uint16_t)new_angle, 100);
                            }
                            /* 车身分级前后移动 */
                            spd = 0;
                            {
                                int16_t ad;
                                ad = ALIGN_DIST_DEAD;
                                if      (error_d > 10) spd = 350;
                                else if (error_d >  5) spd = 250;
                                else if (error_d > ad) spd = 200;
                                else if (error_d < -10) spd = -350;
                                else if (error_d <  -5) spd = -250;
                                else if (error_d < -ad) spd = -200;
                            }
                            {
                                int jj;
                                for (jj = 0; jj < 4; jj++)
                                    target_speed[jj] = spd;
                            }
                            if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                                printf("Place5.1: cx=%d dist=%d err_d=%d spd=%d\r\n",
                                       (int)omv.cx, (int)omv.distance_cm,
                                       (int)error_d, (int)spd);
                                dbg_tick = (uint16_t)g_sys_tick;
                            }
                            if (abs_x <= 10 && (abs_d <= ALIGN_DIST_DEAD || omv.distance_cm <= 4)) {
                                qr_aligned_cnt++;
                                if (qr_aligned_cnt >= 3) {
                                    motor_enable = 0;
                                    {
                                        int jj;
                                        for (jj = 0; jj < 4; jj++) {
                                            target_speed[jj] = 0;
                                            PID_Reset(&pid_motor[jj]);
                                            Motor_SetSpeed(jj, 0);
                                        }
                                    }
                                    printf("Place aligned: cx=%d dist=%d\r\n",
                                           (int)omv.cx, (int)omv.distance_cm);
                                    nodata_start = 0;
                                    scan_step    = 0;
                                    scan_tick    = 0;
                                    qr_phase = 6;
                                    qr_phase_tick = g_sys_tick;
                                }
                            } else {
                                qr_aligned_cnt = 0;
                            }
                        }
                        qr_phase_tick = g_sys_tick;
                    }
                    else
                    {
                        /* 丢目标: 停车 + 腰座扫描恢复 */
                        {
                            int jj;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                        }
                        if (nodata_start == 0)
                            nodata_start = (uint16_t)g_sys_tick;
                        {
                            uint16_t scan_wait;
                            float scan_target;
                            scan_wait = (qr_align_phase == 1) ? 300 : 150;
                            if ((uint16_t)(g_sys_tick - nodata_start)
                                <= scan_wait)
                                break;
                            if (scan_step == 0)
                                scan_target = qr_waist_center + 10.0f;
                            else if (scan_step == 1)
                                scan_target = qr_waist_center - 10.0f;
                            else
                                scan_target = qr_waist_center;
                            if ((uint16_t)(g_sys_tick - scan_tick) > 100) {
                                scan_step = (scan_step + 1) % 3;
                                scan_tick = (uint16_t)g_sys_tick;
                            }
                            if (scan_target < 0.0f)   scan_target = 0.0f;
                            if (scan_target > 180.0f) scan_target = 180.0f;
                            smooth_Settarget(0, (uint16_t)scan_target, 300);
                        }
                        if ((uint16_t)(g_sys_tick - dbg_tick) >= 10) {
                            printf("Place5: scan lost\r\n");
                            dbg_tick = (uint16_t)g_sys_tick;
                        }
                    }

                    /* 10s 硬超时 → 强制放置 */
                    if (g_sys_tick - qr_phase2_start_tick > 1000) {
                        printf("Place align timeout (10s), force place\r\n");
                        motor_enable = 0;
                        {
                            int jj;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                        }
                        nodata_start = 0;
                        scan_step    = 0;
                        scan_tick    = 0;
                        qr_scan_started = 0;
                        qr_phase = 6;
                        qr_phase_tick = g_sys_tick;
                    }
                }
                break;

            case 6:  /* 执行放置 */
                {
                    uint16_t saved_waist;
                    motor_enable = 0;
                    {
                        int jj;
                        for (jj = 0; jj < 4; jj++) {
                            target_speed[jj] = 0;
                            Motor_SetSpeed(jj, 0);
                        }
                    }
                    Delay_ms(200);
                    printf("Action: PLACE at box (color=%d)\r\n",
                           (int)qr_target_color);
                    saved_waist = act_angle[ACT_APPROACH_WAIST];
                    act_angle[ACT_APPROACH_WAIST] =
                        (uint16_t)smooth_GetCurrentAngle(0);
                    if (ArmSM_RequestPlace(110)) {
                        Action_Place(act_angle[ACT_PLACE_WAIST]);
                        ArmSM_NotifyComplete();
                    } else {
                        Action_Place(act_angle[ACT_PLACE_WAIST]);
                        ArmSM_NotifyComplete();
                    }
                    act_angle[ACT_APPROACH_WAIST] = saved_waist;
                    OpenMV_FlushRx();
                    OpenMV_SendCmd("MODE,QRCODE");
                    qr_cooldown_end = g_sys_tick + 1500;
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    /* 起步预加载 */
                    target_speed[0] = 180;
                    target_speed[1] = 180;
                    target_speed[2] = 180;
                    target_speed[3] = 170;
                    qr_is_place = 0;    /* 重置，下次为抓取 */
                    /* 推进任务序列 */
                    mission_idx++;
                    if (mission_idx >= MISSION_COUNT) {
                        mission_done = 1;
                        printf("ALL DONE: %d objects placed.\r\n",
                               MISSION_COUNT);
                    } else {
                        printf("Mission->%d: next class_id=%d\r\n",
                               (int)mission_idx,
                               (int)mission_class[mission_idx]);
                    }
                    qr_phase = 0;
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                    dbg_tick     = 0;
                    printf("Place done, resume line follow\r\n");
                }
                break;

            default:
                qr_phase = 0;
                break;
            }
        }

        // 心跳: LED每秒翻转一次
        {
            static uint32_t tick = 0;
            tick++;
            if (tick > 500000) {
                tick = 0;
                GPIOF->ODR ^= GPIO_Pin_9;
            }
        }

        // CSV数据上报
        {
            static uint32_t last_send_tick = 0;
            if (g_sys_tick - last_send_tick >= 5)
            {
                last_send_tick = g_sys_tick;
                uart_set_bt_output(0);
                /* sendPIDDataToUART(); */
            }
        }

        OpenMV_PollLine();

        // AGV task navigation state machine (legacy)
        TaskNav_Process();
        VisionTask_Process(&vision_task);

        TaskQueue_Process(&task_queue);

        if (BT_GetLine(cmd, sizeof(cmd)))
        {
            uart_set_bt_output(1);
            Commend_Parse(cmd);
            uart_set_bt_output(0);
        }
    }
	}
}
