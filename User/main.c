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
#include <stdio.h>
#include <string.h>

uint8_t KEYNUM;
uint16_t NUM;

#define MODE_MANUAL       0
#define MODE_LINE_FOLLOW  1
#define MODE_POSITION     2
/* MODE_VISUAL_SERVO 3 is defined in visual_servo.h */

/* ======  ======
   1 = ,  MANUAL , AI
       /
   0 =  (+AI) */
#define TEST_MODE 0
#define SKIP_DOCK 1   /* 调试开关: 1=上电跳过对接(直接进回轨 dock_state7 + Station0/1), 0=正常对接流程 */

volatile uint8_t work_mode = MODE_MANUAL;
volatile uint8_t motor_enable = 0;
uint8_t g_delivery_mode = 0;  /* delivery: 1= */
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
volatile uint8_t trailer_docked = 0;   /* 从车已挂接: 1=完成 */
extern volatile uint8_t trailer_dock_flag;  /* ISR 十字路口触发 */
volatile uint8_t dock_test_flag = 0;   /* dock_test: 1=车已对准, 直接进倒车对接 */
volatile uint8_t dock_tune_flag = 0;   /* dock_tune: 1=自动循环对接调参 (不挂钩) */
volatile uint8_t dock_tune_on   = 0;   /* 调参循环模式: 到达38cm后前进离开再倒车 */
volatile uint8_t telem_enable   = 0;   /* telem: 1=遥测流开启(tuner用), 默认关防刷屏 */
volatile uint8_t lf_dbg_enable  = 0;   /* lfdbg: 1=巡线调试打印(传感器/速度/PID输出) */
extern int16_t last_pid_setpoint[4];   /* Timer.c 速度环记录 */
extern int16_t last_pid_speed[4];
extern int16_t last_pid_output[4];
/* 对接状态机全局 (dockstat 调试命令可读) */
volatile uint8_t  dock_state = 0;      /* 0=idle 1=右拐 2=找码 3=倒车 4=挂钩 5=完成 6=调参离开 */
volatile uint32_t dock_tick = 0;
volatile uint8_t  dock_lost = 0;
volatile uint8_t  dock_offline_cnt = 0;
volatile uint8_t  dock_tune_once = 0;  /* dock_tune 初始化一次性标志 (退出时复位) */
volatile uint8_t  cross_dock_enable = 0; /* crossdock: 1=启用十字路口自动对接 (boot 开跑置1, crossdock 0/1 切换) */
volatile uint8_t  dock_done_once = 0;    /* dock_state==5 一次性标志 (run/restart 复位后可重新开跑) */
volatile uint8_t  trace_enable = 0;      /* trace: 1=周期性打印dock/电机状态(定位用) */
/* 十字路口原地右转90° (dock_state 1): 停车→原地差速右转90°→停→转腰座对接.
   停止: 先"深离开"(在线≤2)再"≥4路在线"(对齐竖线); 1.5s最小旋转防误判; 4s兜底; 8s安全超时 */
#define DOCK_PIVOT_SPD     195    /* 原地右转开环速度(左+195/右-195, 用户要求: 200→195) */
#define DOCK_PIVOT_MIN     150    /* 最小旋转时间 1.5s 后才允许判90° (防起步传感器抖动误停) */
#define DOCK_PIVOT_CAP     330    /* 时间兜底 3.3s (用户要求: 3.5s减0.2s) */
#define DOCK_PIVOT_TO      800    /* 安全超时 8s (CAP 之后基本不会到) */
static uint8_t  dock_pv_phase = 0;    /* dock_state1: 0=停车 1=旋转 */
static uint8_t  dock_pv_dropped = 0;  /* 旋转中已"深离开"0° (在线≤2路) */
volatile int16_t  dock_last_cx = -1;     /* 最后有效码 cx (trace 显示, 不受 GetData 消费影响) */
volatile int16_t  dock_last_dist = -1;   /* 最后有效码 dist */
#define DOCK_WAIST_ALIGN_DIR   1         /* 腰座对准方向: trace 实证 waist↑→cx↑(码更右), 码偏右应减小waist → +1 */
#define DOCK_RR_PCT            7         /* 右后轮开环补偿(比例式): 右后轮无编码器开环, 按 |target_speed[3]| 的 7% 反向补偿
                                           (倒车时右后轮稍快, 减速 7% 与左轮一致); 全速度段比例恒定.
                                           若车仍向右偏就减小, 向左偏就加大 */
#define DOCK_INNER_OPENLOOP    1         /* 倒车内环开环开关(2026-08-24): 1=视觉伺服主回路保留, 只跳过编码器速度内环
                                           (编码器方向乱/接触不良时速度环正反馈→车原地转不后退); 0=恢复速度闭环 */
#define DOCK_RF_PCT            15        /* 右前轮(电机1)倒车补偿(比例式): 右前轮低速时实际出力偏小
                                           (日志 spd=-51 右前/右后明显低于左轮 -76, 电机死区附近跑不动),
                                           倒车时按 |target_speed[1]| 的 15% 加大右前轮速度(更负=更有力).
                                           只补偿倒车段(负速度), 不影响正向前进 */
volatile uint8_t qr_trig_flag = 0;   /* : QR */
/* AI 流水线站/目标序号 (全局化: SKIP_DOCK 启动可预设起始站, 原为主循环块内 static) */
uint8_t station_idx = 0;    /* 0=Station0(抓取放从车) 1=Station1(交付) */
uint8_t mission_idx = 0;    /* 当前目标序号 0..MISSION_COUNT-1 */
volatile uint8_t ai_dbg_stat = 0;    /* ai_stat: dump AI pipeline state */
volatile uint8_t ai_dbg_skip = 0;    /* ai_skip: skip current target */
volatile uint8_t ai_dbg_go   = 0;    /* ai_go: jump to phase */
volatile uint8_t ai_dbg_phase = 0;   /* ai_go target phase */
volatile uint8_t ai_restart_flag = 0; /* restart: reset AI pipeline */
volatile uint8_t flow_dbg = 0;        /* flow: dump whole run-flow state (main loop handler) */
volatile uint8_t flow_stage = 1;      /* LED 阶段指示: 1=巡线 2=对接 3=工位 4=全部完成 (无串口也可见) */
static  uint8_t dock_st2_init = 0;    /* dock_state==2 一次性初始化标志 (全局, 便于复位) */
static  uint32_t dock_st2_start = 0;  /* dock_state==2 进入时刻 (30s 总超时用) */
static  uint8_t dock_st7_init = 0;    /* dock_state==7(回轨) 一次性初始化 */
static  uint8_t dock_st7_phase = 0;   /* dock_state==7: 0=脱线直行 1=原地左转 */

// LEDn
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

// 
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
    SysTick_Init();  // 1ms SysTick
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    DBGMCU->CR &= ~(DBGMCU_CR_TRACE_IOEN | (0x3 << 6));

    Motor_EmergencyBrake();

    LED_Init();
    GPIO_SetBits(GPIOF, GPIO_Pin_9);  // LED off initially

    uart_Init();
    printf("a\r\n");        //  a
    /* OLEDI2C */
    /* OLED_Init(); */
    /* OLED_Clear(); */
    /* OLED_ShowString(0,0,(u8*)"Init...",16,1); */
    /* BT_Init(); -- , USART2, UART5(PC12/PD2) */
    OpenMV_Init();

    Motor_Init();
    PWM_init();
    Delay_ms(200);          // PWM[
    Servo_Init();
    smooth_Init();
    ArmSM_Init();
    Action_ParamInit();

    Encoder_Init();
    Line_sensor_Init();

    /* 视觉伺服: tag_id=1=从车AprilTag, 目标40cm(触发到位), 基准160 (负重太慢再加力).
       横向 P-only kp=1.0 + 死区±3px: 轮子差速转向要有劲 (kp=0.8 时转向偏软,
       用户反馈"差速旋转太小"); 腰座在 dist>45cm 时负责粗对准,
       45cm 内交给轮子差速, 防腰座棘轮顶限位.
       纵向 P-only kp=1.5. I/D 全 0: 主循环只在 30fps 新帧时调用 (dt 实为 33ms 非 0.01),
       微分项被放大 3.3 倍会加剧甩头, 积分在横向上会越积越偏 */
    VisualServo_Init(&visual_servo, 1, 40,
        160, 1.0f, 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.01f);
    TaskQueue_Init(&task_queue);

    lineFollow_Init(&line_follow, 180, 180.0, 2.0, 2.0, 0.01);
    PID_Init(&pid_motor[0], PID_INCREMENTAL, 5.5, 0.16, 0.0, 300, 1000, 300);  // 
    PID_Init(&pid_motor[1], PID_INCREMENTAL, 5.5, 0.16, 0.0, 300, 1000, 300);  // 
    PID_Init(&pid_motor[2], PID_INCREMENTAL, 5.0, 0.14, 0.0, 300, 1000, 300);  // 
    PID_Init(&pid_motor[3], PID_INCREMENTAL, 5.0, 0.14, 0.0, 300, 1000, 300);  //  
    for(i=0; i<4; i++)
        PID_Init(&pid_positon[i], PID_POSITION, 0.5, 0.001, 0, 200, 600, 300);

    Timer_Init();
    PVD_Init();         // VDD<2.9V

    // PID
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

#if !TEST_MODE
    // ======  ======
    work_mode = MODE_LINE_FOLLOW;
    motor_enable = 1;
    Action_Reset();
    while (!Action_ISdle());

    // ======  ======
    OpenMV_SendCmd("MODE,QRCODE");  /* QR */

    /* ====== 开跑: 整流程自动启动 ======
       line-follow -> 十字路口(5路全踩) -> 右拐 -> 脱线对准
       -> AprilTag 倒车对接 -> 挂钩 -> Station0(抓3件放从车)
       -> Station1(从车取件交付) -> 完成 LED 三闪 */
    cross_dock_enable = 1;   /* 十字路口自动对接默认开启, crossdock 0/1 可关 */
    flow_stage = 1;          /* LED 慢闪 = 巡线阶段 */
#if SKIP_DOCK
    /* 调试: 上电默认已勾住从车 — 钩子保持勾住(10°夹紧)状态, 不转腰座/不重挂, 直接巡线进 Station0/1.
       (车需手动挂好从车; 上电只把钩子舵机保持勾住角度, 腰座朝前) */
    cross_dock_enable = 0;
    smooth_Settarget(SERVO_HOOK, ARM_HOME_HOOK, 500);   /* 钩子保持勾住(夹紧) */
    OpenMV_SendCmd("MODE,QRCODE");   /* 确保 OpenMV 在 QRCODE 模式, Station0 靠 QR 触发 */
    trailer_docked = 1;        /* 默认已勾住从车 */
    dock_done_once = 1;        /* 对接完成: dock_state5 不再转腰座/进回轨 */
    dock_state = 5;            /* 完成态: AI 流水线激活 → Station0/1 */
    encoder_max[1] = 0;        /* 右前开环(编码器real乱跳→PID猛反→乱转, lfdbg证实) */
    encoder_max[2] = 0;        /* 左后开环(编码器接触不良 real反向→PID输出反→左后倒着跑) */
    work_mode = MODE_LINE_FOLLOW;
    motor_enable = 1;
    target_speed[0] = 180; target_speed[1] = 180;
    target_speed[2] = 180; target_speed[3] = 170;

    /* ===== START_AT_STATION1: 上电直接进 Station1(交付), 跳过 Station0 抓取与巡线 =====
       0=上电巡线跑全流程(Station0->Station1); 1=原地等待5s自动触发Station1.
       车需摆放在 Station1 二维码正前方 15~40cm 对准; 上电原地等待,
       5s 后 (qr_phase=0 内 g_sys_tick>=500) 自动触发 approach -> 170°定位取件 -> 交付,
       全程无需任何串口命令。跑完 3 件(红黄绿)自动 mission_done 停车。 */
    #define START_AT_STATION1  0
    #if START_AT_STATION1
        station_idx   = 1;              /* 从交付站开始 */
        mission_idx   = 0;              /* 第一件: 红(class0) */
        work_mode     = MODE_MANUAL;    /* 不巡线: 原地等待自动触发 */
        target_speed[0] = 0; target_speed[1] = 0;
        target_speed[2] = 0; target_speed[3] = 0;
        qr_trig_flag  = 1;              /* 上电自动触发 Station1 */
        printf("START_AT_STATION1: station=%d mission=%d, 5s auto-trigger\r\n",
               (int)station_idx, (int)mission_idx);
    #endif

    printf("===== AGV RUN FLOW READY (SKIP DOCK, HOOKED) =====\r\n");
    printf("flow: line-follow -> Station0 -> Station1\r\n");
#else
    printf("===== AGV RUN FLOW READY =====\r\n");
    printf("flow: line-follow -> crossroad dock -> Station0 -> Station1\r\n");
    printf("cross_dock_enable=1 (crossroad auto-dock ON)\r\n");
#endif
#else
    // ====== : ,  ======
    work_mode = MODE_MANUAL;
    motor_enable = 0;
    printf("TEST MODE: manual control only\r\n");
    Action_Reset();
    while (!Action_ISdle());
    smooth_Settarget(0, ARM_HOME_WAIST,   800);
    smooth_Settarget(1, ARM_HOME_SHOULDER, 800);
    smooth_Settarget(2, ARM_HOME_ELBOW,    800);
    printf("Arm reset done, ready for manual commands.\r\n");
#endif
    LedBlink(3);        // LED3 = MCU

    //  USB-TTL ISR 
    while (uart_rb_getchar() >= 0);

    // 
    {
        uint32_t loop_start_tick;
        loop_start_tick = g_sys_tick;

    while (1)
    {
        /* ++ */
        {
            static uint8_t  line_idx = 0;
            static char     line_buf[64];
            int ch_int;
            uint8_t ch;
            uint32_t uptime;

            uptime = g_sys_tick - loop_start_tick;

            /* (ISRprintfE) */
            while ((ch_int = uart_rb_getchar()) >= 0)
            {
                ch = (uint8_t)ch_int;
                /* 3,  */
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
                /* ++, $TAGOpenMV */
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

            /* RX  ISR  , 
             * ( RXNE  ISR ) */
        }

        // ======  (,  QR ) ======
#if 0
        {
            static uint8_t  auto_phase = 0;  /* 0=, 1=, 2= */
            static uint32_t phase1_start_tick = 0;  /*  */
            switch (auto_phase) {
                case 0:  /* : 5s   */
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
                case 1:  /* 17s   */
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
                default:  /* ,  */
                    break;
            }
        }
#endif

        /*  OpenMV ,  AI  */
        OpenMV_PollLine();

        // ====== AI : AI ======
        // : QR, AI ()
#if !TEST_MODE

        #define STATION_MODE      1   /* 1=() 0= */
        /*    */
        #define GRASP_TARGET_CM    6    /*  (cm) */
        #define PLACE_TARGET_CM    6    /*  (cm) */
        #define ALIGN_DIST_DEAD    2    /*  (cm) */
        /* -:  cx 
           160=, (180), (140) */
        #define GRASP_CX_TARGET   190   /* ,  */
        #define TRAILER_PLACE_CX  210   /* cx:  */
        #define MISSION_COUNT      3   /*  */

        {
            /* ---- : (0)(2)(1) ---- */
            static int8_t  mission_class[MISSION_COUNT] = {0, 2, 1};
            /* (1= 2= 3=), =color-1 */
            static int16_t color_cx[3]        = {210, 170, 190};  /*   :  */
            static int16_t color_grasp_dist[3] = {  6,   6,   6};  /*   : 6cm, cxdist */
            static int16_t color_conf_min[3]  = { 30,  30,  50};  /* AI conf 阈值: 红45→35→30(仍不稳), 绿30, 黄50 */
            static int16_t color_scan_deg[3] = {  5,   5,   5};  /* O: 5 */
            /* 识别色卡时腰座写死角度: [color_id] = 角度 (1红=90, 2绿=100, 3黄=80) */
            static int16_t identify_waist[4]  = {  0,  90, 100,  80 };
            /* 交付(放置)腰座写死角度: [color_id] = 角度 — 三个放不同位置
               (1红=90中间, 2绿=110, 3黄=70) */
            static int16_t deliver_waist[4]   = {  0,  90, 110,  70 };
            static uint8_t mission_done = 0;
            /* station_idx / mission_idx 已全局化(文件顶部), 启动可预设起始站 */
            static uint8_t total_stations = 2;  /* : 0=, 1= */
            static uint32_t mission_last_print = 0;
            static uint8_t  qr_phase = 0;      /* 0= 1=AI 2= 3= */
            static uint32_t qr_phase_tick = 0;
            static uint32_t qr_cooldown_end = 0; /* ,  */
            static uint8_t  qr_is_place = 0;   /* 0= 1=  STATION_MODEPhase7/9,  */
            static uint8_t  qr_target_color = 1; /* AI */
            static uint8_t  qr_aligned_cnt = 0;  /*  */
            static uint8_t  qa_window = 0;       /* QuickAlign: bit0=, 5 */
            static float    qa_anchor = 90.0f;   /* QuickAlign 锚点: 进入时角度, 限幅±30防失控转远 */
            static uint8_t  qr_scan_started = 0; /* 0= 1= */
            static float    qr_waist_center = 90.0f; /* AI,  (270) */
            /* Bug#1: Phase 2  () */
            static uint8_t  qr_phase2_arm_done = 0; /* Phase2.0  */
            /* 识别确认计数: 累计可见 tick, 防识别瞬间跳过直接转170° */
            static uint16_t id_hit_cnt = 0;
            /* Bug#2: ,  */
            static uint16_t nodata_start = 0;
            static uint8_t  scan_step   = 0;
            static uint16_t scan_tick   = 0;
            static uint16_t dbg_tick    = 0;  /* printf  */
            /* Bug#4: Phase 1  */
            static uint8_t  phase1_scan_step = 0;
            static uint16_t phase1_scan_tick = 0;
            /* Phase 9 skip-pick: 1Phase6Phase9 */
            static uint8_t  qr_skip_pick = 0;
            /* Phase 4 m */
            static uint32_t qr_place_phase_start = 0;
            OpenMV_CLSData  cls;
            OpenMV_Data     omv;
            int j;

            /* ====== AI debug commands (set by command.c) ====== */
            if (ai_dbg_stat) {
                ai_dbg_stat = 0;
                printf("[AI] phase=%d mission=%d/%d station=%d/%d"
                       " class=%d is_place=%d\r\n",
                       (int)qr_phase, (int)mission_idx, MISSION_COUNT,
                       (int)station_idx, total_stations,
                       (int)mission_class[mission_idx],
                       (int)qr_is_place);
                printf("[AI] scan: deg=%d waist=%.0f started=%d"
                       " step=%d tick=%d\r\n",
                       (int)color_scan_deg[mission_class[mission_idx]],
                       (double)qr_waist_center, (int)qr_scan_started,
                       (int)phase1_scan_step, (int)phase1_scan_tick);
                printf("[AI] align: phase=%d cnt=%d tgt_cx=%d"
                       " tgt_dist=%d conf_min=%d\r\n",
                       (int)qr_aligned_cnt,
                       (int)color_cx[qr_target_color - 1],
                       (int)color_grasp_dist[qr_target_color - 1],
                       (int)color_conf_min[mission_class[mission_idx]]);
            }
            /* ====== flow: 全流程状态一屏 dump (dock + AI + 电机) ====== */
            if (flow_dbg) {
                flow_dbg = 0;
                printf("[FLOW] wm=%d en=%d cross=%d dock_st=%d"
                       " lost=%d tune=%d docked=%d\r\n",
                       (int)work_mode, (int)motor_enable,
                       (int)cross_dock_enable, (int)dock_state,
                       (int)dock_lost,
                       (int)dock_tune_on, (int)trailer_docked);
                printf("[FLOW] phase=%d m=%d/%d st=%d/%d done=%d"
                       " place=%d color=%d dlv=%d\r\n",
                       (int)qr_phase, (int)mission_idx, MISSION_COUNT,
                       (int)station_idx, total_stations,
                       (int)mission_done, (int)qr_is_place,
                       (int)qr_target_color, (int)g_delivery_mode);
                printf("[FLOW] spd=%d,%d,%d,%d waist=%.0f cls=[%d,%d,%d]\r\n",
                       (int)target_speed[0], (int)target_speed[1],
                       (int)target_speed[2], (int)target_speed[3],
                       (double)smooth_GetCurrentAngle(0),
                       (int)mission_class[0], (int)mission_class[1],
                       (int)mission_class[2]);
            }
            if (ai_dbg_skip) {
                ai_dbg_skip = 0;
                printf("[AI] skip: advancing mission_idx %d->%d\r\n",
                       (int)mission_idx, (int)(mission_idx + 1));
                mission_idx++;
                if (mission_idx >= MISSION_COUNT) {
                    printf("[AI] skip: all objects done,"
                           " next station\r\n");
                    mission_idx = 0;
                    station_idx++;
                    if (station_idx >= total_stations) {
                        mission_done = 1;
                        flow_stage = 4;     /* LED 常亮 = 全部完成 */
                        printf("[AI] ALL STATIONS DONE\r\n");
                    }
                }
                OpenMV_SendCmd("MODE,QRCODE");
                LineFollow_Reset(&line_follow);
                work_mode = MODE_LINE_FOLLOW;
                motor_enable = 1;
                target_speed[0] = 180; target_speed[1] = 180;
                target_speed[2] = 180; target_speed[3] = 170;
                qr_cooldown_end = g_sys_tick + 500;
                qr_phase = 0;
                qr_scan_started = 0;
                nodata_start = 0;
                scan_step    = 0;
                scan_tick    = 0;
                dbg_tick     = 0;
                phase1_scan_step = 0;
                phase1_scan_tick = 0;
                id_hit_cnt = 0;
                qr_is_place = 0;
            }
            if (ai_dbg_go) {
                uint8_t target = ai_dbg_phase;
                ai_dbg_go = 0;
                printf("[AI] go: phase %d -> %d\r\n",
                       (int)qr_phase, (int)target);
                qr_phase = target;
                qr_phase_tick = g_sys_tick;
                qr_scan_started = 0;
                nodata_start = 0;
                scan_step    = 0;
                scan_tick    = 0;
                dbg_tick     = 0;
                phase1_scan_step = 0;
                phase1_scan_tick = 0;
                id_hit_cnt = 0;
            }

            /* ====== restart: emergency reset all AI pipeline state ====== */
            if (ai_restart_flag) {
                int rj;
                ai_restart_flag = 0;
                printf("[AI] RESTART: resetting pipeline state\r\n");
                motor_enable = 0;
                for (rj = 0; rj < 4; rj++) {
                    target_speed[rj] = 0;
                    PID_Reset(&pid_motor[rj]);
                    Motor_SetSpeed(rj, 0);
                }
                qr_phase = 0;
                qr_phase_tick = 0;
                qr_cooldown_end = g_sys_tick + 100; /* 1s mini cooldown */
                qr_is_place = 0;
                qr_aligned_cnt = 0;
                qa_window = 0;
                qr_scan_started = 0;
                nodata_start = 0;
                scan_step    = 0;
                scan_tick    = 0;
                dbg_tick     = 0;
                phase1_scan_step = 0;
                phase1_scan_tick = 0;
                qr_phase2_arm_done = 0;
                qr_skip_pick = 0;
                id_hit_cnt = 0;
                /* 工位/任务计数: 回到 Station0 第 1 件 */
                mission_idx = 0;
                station_idx = 0;
                mission_done = 0;
                g_delivery_mode = 0;
                /* dock state machine disabled (go straight to grasp/place): mirror SKIP_DOCK boot state */
                dock_state = 5;
                trailer_dock_flag = 0;
                trailer_docked = 1;
                dock_done_once = 1;
                dock_tune_on = 0;
                dock_tune_once = 0;
                dock_lost = 0;
                dock_offline_cnt = 0;
                dock_tick = 0;
                dock_last_cx = -1;
                dock_last_dist = -1;
                cross_dock_enable = 0;
                dock_st2_init = 0;
                dock_pv_phase = 0;       /* 原地右转状态复位 */
                dock_pv_dropped = 0;
                dock_st7_init = 0;       /* 回轨状态复位 */
                dock_st7_phase = 0;
                flow_stage = 1;          /* LED 回到巡线阶段 */
                /* 机械臂复位 (空闲时才做, 防打断抓取中的动作) */
                if (!ArmSM_IsBusy()) {
                    Action_Reset();
                }
                OpenMV_SendCmd("MODE,QRCODE");
                LineFollow_Reset(&line_follow);
                work_mode = MODE_LINE_FOLLOW;
                motor_enable = 1;
                target_speed[0] = 180; target_speed[1] = 180;
                target_speed[2] = 180; target_speed[3] = 170;
                printf("[AI] RESTART: flow restarted, line-follow resumed\r\n");
            }

            /* dock_tune 调参期间冻结 AI 自动任务, 防误触夹爪/切模式;
               (2026-08-24 逻辑冲突修复) AI 流水线只在 dock 完成后(dock_state==5)激活 —
               原逻辑 AI 与 dock 状态机并存无互斥: dock 收尾切 QRCODE 后 AI 立即被 QR 触发
               approach 前进, 抢 motor/OpenMV → 车往前跑/乱跑 */
            if (!dock_tune_on) {   /* dock disabled: no longer require dock_state==5 */
            switch (qr_phase) {
            case 0:  /* , QR    15cm  AI */
                {
                    OpenMV_QRData qr;
                    OpenMV_Data     omv;
                    uint32_t        app_start;
                    int             app_done;

                    /* Bugfix H1: omvprintf */
                    omv.tag_id = -1;
                    omv.cx = 0;
                    omv.cy = 0;
                    omv.distance_cm = 0;
                    omv.angle_deg = 0;
                    omv.pixel_width = 0;
                    omv.timestamp = 0;
                    omv.fresh = 0;

                    /* : , QR */
                    if (mission_done) {
                        /* 全部任务完成: 继续巡线直行, 直到没有黑线(轨道尽头)再停车 */
                        static uint8_t  done_blinked = 0;
                        static uint16_t no_line_cnt = 0;
                        bool ls[5];
                        uint8_t any_line;
                        int lj;
                        /* 确保巡线模式 (交付完可能在别的模式) */
                        if (work_mode != MODE_LINE_FOLLOW || !motor_enable) {
                            OpenMV_SendCmd("MODE,QRCODE");
                            LineFollow_Reset(&line_follow);
                            work_mode = MODE_LINE_FOLLOW;
                            motor_enable = 1;
                            target_speed[0] = 180; target_speed[1] = 180;
                            target_speed[2] = 180; target_speed[3] = 170;
                        }
                        /* 检测没线: 5路全白连续 50 tick(0.5s) -> 停车 */
                        LineSensor_Read(ls);
                        any_line = 0;
                        for (lj = 0; lj < 5; lj++)
                            if (ls[lj]) any_line = 1;
                        if (any_line) {
                            no_line_cnt = 0;
                        } else {
                            no_line_cnt++;
                            if (no_line_cnt >= 50) {
                                motor_enable = 0;
                                work_mode = MODE_MANUAL;
                                for (lj = 0; lj < 4; lj++) {
                                    target_speed[lj] = 0;
                                    PID_Reset(&pid_motor[lj]);
                                    Motor_SetSpeed(lj, 0);
                                }
                                if (!done_blinked) {
                                    done_blinked = 1;
                                    LedBlink(3);  /* LED 3-blink = mission complete */
                                }
                                if (g_sys_tick - mission_last_print > 1000) {
                                    mission_last_print = g_sys_tick;
                                    printf("Mission complete. %d objects done,"
                                           " end of line.\r\n", MISSION_COUNT);
                                }
                            }
                        }
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }
                    /* 5 */
                    if (g_sys_tick < 500)
                        break;
                    /* QR,  */
                    if (g_sys_tick < qr_cooldown_end) {
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }
                    /* QR */
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
                    /* , QR */
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
                                if (omv.distance_cm <= GRASP_TARGET_CM) {  /* Bugfix H2: 12 */
                                    app_done = 1;
                                } else {
                                    for (j = 0; j < 4; j++)
                                        target_speed[j] = 80;
                                }
                            } else {
                                lost_cnt++;
                                for (j = 0; j < 4; j++)
                                    target_speed[j] = 0;
                                /* 5=,  */
                                if (lost_cnt >= 5) {
                                    printf("QR: lost (too close?)\r\n");
                                    app_done = 1;
                                }
                            }
                            Delay_ms(100);
                        }
                    }
                    /*  */
                    motor_enable = 0;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        PID_Reset(&pid_motor[j]);
                        Motor_SetSpeed(j, 0);
                    }
                    printf("QR: stop at %d cm\r\n", (int)omv.distance_cm);
                    Delay_ms(300);
                    /*  */
                    smooth_Settarget(1, 50, 600);
                    smooth_Settarget(2, 160, 600);
                    smooth_Settarget(0, 90, 600);
                    /* = = */
                    if ((station_idx & 1) == 0 && !g_delivery_mode) {
                        /* Station 0: 红是深红色, 用 COLOR,1 识别(深红阈值可靠);
                           AI 对深红 conf 不稳(实测反复超时), 放弃 AI 识别红 */
                        qr_target_color = 1;
                        OpenMV_FlushRx();
                        OpenMV_SendCmd("MODE,COLOR,1");
                        /* 识别姿势沿用上方公共姿势(腰座90/大臂50/小臂160), 不覆盖 */
                        qr_scan_started = 0;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = 0;
                        qr_phase2_arm_done = 0;  /* 0=识别阶段 */
                        id_hit_cnt = 0;
                        qr_phase = 1;
                        printf("Station %d: pickup red via COLOR,1\r\n",
                               (int)station_idx);
                    } else {
                        /* Station 1: 90° 识别颜色(COLOR) -> 170° 从车定位夹取 -> 回90°交付 */
                        /* 目标颜色 = mission_class+1 (class0红->color1, class2黄->color3, class1绿->color2) */
                        {
                            char cbuf[16];
                            qr_target_color = (uint8_t)(mission_class[mission_idx] + 1);
                            if (qr_target_color < 1) qr_target_color = 1;
                            if (qr_target_color > 3) qr_target_color = 3;
                            sprintf(cbuf, "MODE,COLOR,%d", (int)qr_target_color);
                            OpenMV_SendCmd(cbuf);
                        }
                        OpenMV_FlushRx();
                        /* 识别姿势: 复位(大臂0/小臂120) + 腰座按颜色写死角(红90/绿95/黄85) */
                        smooth_Settarget(0, (uint16_t)identify_waist[qr_target_color], 600);
                        smooth_Settarget(1, ARM_HOME_SHOULDER, 600);
                        smooth_Settarget(2, ARM_HOME_ELBOW,    600);
                        qr_scan_started = 0;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = 0;
                        qr_phase2_arm_done = 0;  /* 0=识别阶段 */
                        id_hit_cnt = 0;
                        qr_phase = 1;
                        printf("Station %d: identify color %d at %d deg\r\n",
                               (int)station_idx, (int)qr_target_color,
                               (int)identify_waist[qr_target_color]);
                    }
                    g_delivery_mode = 0;  /* Bugfix H3: delivery,  */
                    qr_phase_tick = g_sys_tick;
                }
                break;

            case 1:  /* 识别(腰座±20°扫描) -> Station0直接抓 / Station1 170°定位 */
                OpenMV_PollLine();

                /* 第一阶段: 识别目标 — 腰座 ±20° 旋转扫描找, 找到后停扫累计确认 */
                if (qr_phase2_arm_done == 0) {
                    /* 腰座到识别位后再开始 */
                    if (!qr_scan_started) {
                        if (smoothservo_IsBusy(0)
                            && g_sys_tick - qr_phase_tick < 200) {
                            break;   /* 等腰座到位, 最多等 2s 兜底 */
                        }
                        if ((station_idx & 1) == 0) {
                            /* Station 0 识别中心: 红90° / 黄110° / 绿50°(物体39-54°) */
                            if (mission_idx == 2 && mission_class[2] == 1)
                                qr_waist_center = 50.0f;
                            else if (mission_class[mission_idx] == 2)
                                qr_waist_center = 110.0f;
                            else
                                qr_waist_center = 90.0f;
                        } else {
                            qr_waist_center =
                                (float)identify_waist[qr_target_color];
                        }
                        qr_scan_started = 1;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = (uint16_t)(g_sys_tick - 31);
                        printf("Identify: center=%.0f deg (color %d),"
                               " scan +-20\r\n",
                               (double)qr_waist_center, (int)qr_target_color);
                    }
                    if (!smoothservo_IsBusy(0)
                        && (uint16_t)(g_sys_tick - phase1_scan_tick) > 60) {
                        uint8_t id_ok;
                        OpenMV_PollLine();
                        id_ok = 0;
                        /* AI 模式: $CLS class_id 匹配 mission_class
                           (Station 0 首次 = MODE,AI) */
                        if (OpenMV_GetCLSData(&cls)
                            && cls.class_id
                               == (int16_t)mission_class[mission_idx]
                            && cls.confidence
                               >= (uint8_t)color_conf_min[
                                    mission_class[mission_idx]]) {
                            id_ok = 1;
                        }
                        /* COLOR 模式: $TAG tag_id 匹配目标颜色
                           (Station 1 / Station 0 下一件 / 绿色走 COLOR) */
                        if (OpenMV_GetData(&omv)
                            && omv.tag_id == qr_target_color
                            && omv.distance_cm > 0) {
                            id_ok = 1;
                        }
                        if (id_ok) {
                            /* 看到目标: 停扫, 保持角度累计 100 tick(1s) 确认 */
                            phase1_scan_tick = (uint16_t)g_sys_tick;
                            if (id_hit_cnt == 0)
                                id_hit_cnt = (uint16_t)g_sys_tick;
                            if ((uint16_t)(g_sys_tick - id_hit_cnt) >= 100) {
                                printf("Identify HIT: station=%d tgt_cls=%d"
                                       " waist=%.0f\r\n",
                                       (int)station_idx,
                                       (int)mission_class[mission_idx],
                                       (double)smooth_GetCurrentAngle(0));
                                if ((station_idx & 1) == 0) {
                                    /* Station 0: 物体在 90° 侧边 —
                                       识别确认后直接 QuickAlign 抓取, 不转 180° */
                                    char cbuf[16];
                                    OpenMV_FlushRx();
                                    sprintf(cbuf, "MODE,COLOR,%d",
                                            (int)qr_target_color);
                                    OpenMV_SendCmd(cbuf);
                                    Delay_ms(100);  /* Bug#6: 模式切换防首帧过期 */
                                    qr_aligned_cnt = 0;
                                    qa_window = 0;
                                    qr_phase = 10;
                                    qr_phase_tick = g_sys_tick;
                                    id_hit_cnt = 0;
                                    break;
                                }
                                /* Station 1: 物体在车尾(170°) —
                                   观测动作(腰座180)后转 170° 定位 */
                                Action_Observe();
                                qr_phase2_arm_done = 1;
                                qr_scan_started = 0;
                                qr_phase_tick = g_sys_tick;
                                phase1_scan_step = 0;
                                phase1_scan_tick = 0;
                                id_hit_cnt = 0;
                                break;
                            }
                        } else {
                            /* 看不到: 丢失 2s 清零, 三角扫描 ±20° 找 */
                            if (id_hit_cnt != 0
                                && (uint16_t)(g_sys_tick - id_hit_cnt) > 200)
                                id_hit_cnt = 0;
                            {
                                float target;
                                int step_n = phase1_scan_step;
                                int tri_idx = (step_n + 1) % 16;
                                int offset;
                                if (tri_idx == 0)
                                    offset = 0;
                                else if (tri_idx <= 4)
                                    offset = tri_idx * 5;
                                else if (tri_idx <= 8)
                                    offset = (8 - tri_idx) * 5;
                                else if (tri_idx <= 12)
                                    offset = (8 - tri_idx) * 5;
                                else
                                    offset = (tri_idx - 16) * 5;
                                target = qr_waist_center + (float)offset;
                                phase1_scan_step++;
                                if (phase1_scan_step >= 16) {
                                    phase1_scan_step = 0;
                                    printf("Identify: scan loop\r\n");
                                }
                                if (target < 0.0f) target = 0.0f;
                                if (target > 270.0f) target = 270.0f;
                                printf("Identify: scan step=%d -> %.0f\r\n",
                                       (int)phase1_scan_step, (double)target);
                                smooth_Settarget(0, (uint16_t)target, 300);
                                phase1_scan_tick = (uint16_t)g_sys_tick;
                            }
                        }
                    }
                    /* 10s 超时: 识别不到 (原15s, 红AI不稳老超时 — 缩短快速fallback) */
                    if (g_sys_tick - qr_phase_tick > 1000) {
                        if ((station_idx & 1) == 0) {
                            /* Station 0: 超时回识别中心(红90/黄110/绿50)再 QuickAlign */
                            char cbuf[16];
                            float fb_a;
                            if (mission_idx == 2 && mission_class[2] == 1)
                                fb_a = 50.0f;    /* 绿 39-54° */
                            else if (mission_class[mission_idx] == 2)
                                fb_a = 110.0f;   /* 黄 >90° */
                            else
                                fb_a = 90.0f;    /* 红 */
                            printf("Identify: timeout, station0 -> "
                                   "back to center, QuickAlign\r\n");
                            smooth_Settarget(0, (uint16_t)fb_a, 600);
                            while (smoothservo_IsBusy(0))
                                Delay_ms(20);
                            Delay_ms(200);
                            OpenMV_FlushRx();
                            sprintf(cbuf, "MODE,COLOR,%d",
                                    (int)qr_target_color);
                            OpenMV_SendCmd(cbuf);
                            Delay_ms(100);
                            qr_aligned_cnt = 0;
                            qa_window = 0;
                            qr_phase = 10;
                            qr_phase_tick = g_sys_tick;
                            id_hit_cnt = 0;
                        } else {
                            /* Station 1: 观测动作后转 170° 定位 */
                            printf("Identify: timeout, go observe+locate\r\n");
                            Action_Observe();
                            qr_phase2_arm_done = 1;
                            qr_scan_started = 0;
                            qr_phase_tick = g_sys_tick;
                            phase1_scan_step = 0;
                            phase1_scan_tick = 0;
                            id_hit_cnt = 0;
                        }
                    }
                    break;
                }

                /* 第二阶段: 170° 从车方向定位色块 */
                {
                    if (!qr_scan_started) {
                        if (smoothservo_IsBusy(0)
                            && g_sys_tick - qr_phase_tick < 200) {
                            break;   /* 等腰座转到180°(车尾) */
                        }
                        qr_waist_center = 180.0f;
                        qr_scan_started = 1;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = (uint16_t)(g_sys_tick - 31);
                        printf("Locate: stop-and-look, center=%.0f\r\n",
                               (double)qr_waist_center);
                    }
                    if (!smoothservo_IsBusy(0)
                        && (uint16_t)(g_sys_tick - phase1_scan_tick) > 60) {
                        OpenMV_PollLine();
                        if (OpenMV_GetData(&omv)
                            && omv.tag_id == qr_target_color
                            && omv.distance_cm > 0) {
                            int16_t ldex = omv.cx - 160;
                            int16_t ldex_abs = (ldex < 0) ? -ldex : ldex;
                            printf("Locate: color=%d cx=%d dist=%d waist=%.0f err=%d\r\n",
                                   (int)omv.tag_id, (int)omv.cx,
                                   (int)omv.distance_cm,
                                   (double)smooth_GetCurrentAngle(0), (int)ldex);
                            if (ldex_abs <= 20) {
                                /* 对准 cx≈160, 定位完成 */
                                printf("Locate HIT: color=%d cx=%d waist=%.0f\r\n",
                                       (int)omv.tag_id, (int)omv.cx,
                                       (double)smooth_GetCurrentAngle(0));
                                smooth_Settarget(0,
                                    (uint16_t)smooth_GetCurrentAngle(0), 40);
                                qr_scan_started = 0;
                                OpenMV_FlushRx();
                                Delay_ms(300);
                                qr_phase = 10;
                                qr_phase_tick = g_sys_tick;
                                qr_aligned_cnt = 0;
                                qa_window = 0;
                                break;
                            }
                            /* 未对准: 腰座微调 (朝 cx=160 方向转) */
                            {
                                float la, ldelta, lnew;
                                la = smooth_GetCurrentAngle(0);
                                ldelta = -(float)ldex * 0.05f;
                                if (ldelta > 5.0f)  ldelta = 5.0f;
                                if (ldelta < -5.0f) ldelta = -5.0f;
                                lnew = la + ldelta;
                                if (lnew < 0.0f)   lnew = 0.0f;
                                if (lnew > 270.0f) lnew = 270.0f;
                                smooth_Settarget(0, (uint16_t)lnew, 100);
                            }
                        } else {
                            /* 看不到目标色块, 三角扫描找 */
                            {
                                float target;
                                int step_n = phase1_scan_step;
                                int tri_idx = (step_n + 1) % 16;
                                int offset;
                                if (tri_idx == 0)
                                    offset = 0;
                                else if (tri_idx <= 4)
                                    offset = tri_idx * 5;
                                else if (tri_idx <= 8)
                                    offset = (8 - tri_idx) * 5;
                                else if (tri_idx <= 12)
                                    offset = (8 - tri_idx) * 5;
                                else
                                    offset = (tri_idx - 16) * 5;
                                target = qr_waist_center + (float)offset;
                                phase1_scan_step++;
                                if (phase1_scan_step >= 16) {
                                    phase1_scan_step = 0;
                                    printf("Locate: scan loop\r\n");
                                }
                                if (target < 0.0f) target = 0.0f;
                                if (target > 270.0f) target = 270.0f;
                                printf("Locate: step=%d -> %.0f\r\n",
                                       (int)phase1_scan_step, (double)target);
                                smooth_Settarget(0, (uint16_t)target, 300);
                                phase1_scan_tick = (uint16_t)g_sys_tick;
                            }
                        }
                    }
                    /* 15s 超时: 170° 找不到 -> fallback QuickAlign */
                    if (g_sys_tick - qr_phase_tick > 1500) {
                        printf("Locate: timeout at 170, grasp anyway\r\n");
                        qr_scan_started = 0;
                        OpenMV_FlushRx();
                        Delay_ms(200);
                        qr_phase = 10;
                        qr_phase_tick = g_sys_tick;
                        qr_aligned_cnt = 0;
                        qa_window = 0;
                    }
                    break;
                }

            case 10: /* QuickAlign: COLOR 色块对准 (腰座到位才微调防过冲) */
                OpenMV_PollLine();
                if (OpenMV_GetData(&omv) && omv.distance_cm > 0) {
                    int16_t error_x, abs_x;
                    int16_t tgt_cx;
                    tgt_cx = 160;  /* 与定位对准目标一致(画面中心) */
                    error_x = omv.cx - tgt_cx;
                    abs_x = (error_x < 0) ? -error_x : error_x;
                    /* 腰座到位才微调+窗口确认: 防连续指令过冲锁错目标
                       (0.08 增益时黄 cx 299→98 跳变=转过冲, 抓空) */
                    if (!smoothservo_IsBusy(0)) {
                        /* 首次进入记录锚点(进入时腰座角度), 后续限幅±30°:
                           防 dist 太近色块抖动时 QA 失控转远(如黄 150°→12°) */
                        if (qa_window == 0 && qr_aligned_cnt == 0)
                            qa_anchor = smooth_GetCurrentAngle(0);
                        /*  */
                        {
                            float cur_a, delta, new_a;
                            cur_a = smooth_GetCurrentAngle(0);
                            delta = -(float)error_x * 0.05f;
                            if (delta > 8.0f)  delta = 8.0f;
                            if (delta < -8.0f) delta = -8.0f;
                            new_a = cur_a + delta;
                            if (new_a > qa_anchor + 30.0f)
                                new_a = qa_anchor + 30.0f;
                            if (new_a < qa_anchor - 30.0f)
                                new_a = qa_anchor - 30.0f;
                            if (new_a < 0.0f)   new_a = 0.0f;
                            if (new_a > 270.0f) new_a = 270.0f;
                            smooth_Settarget(0, (uint16_t)new_a, 150);
                        }
                        /* : 53,  */
                        {
                            uint8_t bits, pass_count;
                            qa_window = (uint8_t)((qa_window << 1) & 0x1F);
                            if (abs_x <= 20) {
                                qa_window |= 1;
                            }
                            bits = qa_window;
                            pass_count = 0;
                            while (bits) { pass_count++; bits &= bits - 1; }
                            qr_aligned_cnt = pass_count;
                            if (pass_count >= 3) {
                                printf("QuickAlign: cx=%d ok (window=0x%02X), grasp\r\n",
                                       (int)omv.cx, (unsigned)qa_window);
                                motor_enable = 0;
                                work_mode = MODE_MANUAL;
                                for (j = 0; j < 4; j++) {
                                    target_speed[j] = 0;
                                    Motor_SetSpeed(j, 0);
                                }
                                qr_phase = 3;
                                qr_phase_tick = g_sys_tick;
                            }
                        }
                    }
                    if ((uint16_t)(g_sys_tick - dbg_tick) >= 50) {
                        printf("QA: cx=%d dist=%d cnt=%d\r\n",
                               (int)omv.cx, (int)omv.distance_cm,
                               (int)qr_aligned_cnt);
                        dbg_tick = (uint16_t)g_sys_tick;
                    }
                }
                /* 8s 超时: 对准不了 -> 盲抓 (红偏太远时 5s 不够, 放宽到 8s) */
                if (g_sys_tick - qr_phase_tick > 800) {
                    printf("QuickAlign: timeout, grasp anyway\r\n");
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        Motor_SetSpeed(j, 0);
                    }
                    qr_phase = 3;
                    qr_phase_tick = g_sys_tick;
                }
                break;

            case 3:  /* :  */
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
                    /* Bug#3:  ArmSM ,  */
                    if (ArmSM_RequestGrasp()) {
                        /* I,  Action_Grasp  */
                        saved_waist = act_angle[ACT_APPROACH_WAIST];
                        act_angle[ACT_APPROACH_WAIST] =
                            (uint16_t)smooth_GetCurrentAngle(0);
                        g_skip_waist_home = 1;  /*  */
                        if (station_idx == 0) {
                            Action_Grasp();
                        } else {
                            Action_GraspFromTrailer();
                        }
                        g_skip_waist_home = 0;
                        act_angle[ACT_APPROACH_WAIST] = saved_waist;
                        ArmSM_NotifyComplete();
                        /*  */
                        OpenMV_FlushRx();
#if STATION_MODE
                        if (station_idx == 0) {
                            /* Station 0:   180Phase7AprilTag */
                            printf("Grasp done, station 0 -> trailer at 180\r\n");
                            qr_phase = 11;
                        } else {
                            /* Station 1:   90Phase9() */
                            printf("Grasp done, station 1 -> deliver at 90\r\n");
                            qr_skip_pick = 1;  /* , Phase9 */
                            qr_target_color = (uint8_t)(mission_class[mission_idx] + 1);
                            qr_phase = 9;
                        }
                        qr_phase_tick = g_sys_tick;
#else
                        /*    */
                        OpenMV_SendCmd("MODE,QRCODE");
                        qr_cooldown_end = g_sys_tick + 500;
                        qr_place_phase_start = g_sys_tick;
                        LineFollow_Reset(&line_follow);
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                        target_speed[0] = 180;
                        target_speed[1] = 180;
                        target_speed[2] = 180;
                        target_speed[3] = 170;
                        qr_is_place = 1;
                        qr_phase = 4;
                        qr_scan_started = 0;
                        nodata_start = 0;
                        scan_step    = 0;
                        scan_tick    = 0;
                        dbg_tick     = 0;
                        qr_phase2_arm_done = 0;
                        phase1_scan_step = 0;
                        phase1_scan_tick = 0;
                        printf("Grasp done, enter trailer grasp pipeline\r\n");
#endif
                    } else {
                        /* : ,  */
                        printf("Action: arm busy, skip grasp, abort\r\n");
                        OpenMV_SendCmd("MODE,QRCODE");
                        qr_cooldown_end = g_sys_tick + 1500; /* 15s */
                        LineFollow_Reset(&line_follow);
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                        target_speed[0] = 180;
                        target_speed[1] = 180;
                        target_speed[2] = 180;
                        target_speed[3] = 170;
                        qr_scan_started = 0;
                        nodata_start = 0;
                        scan_step    = 0;
                        scan_tick    = 0;
                        dbg_tick     = 0;
                        qr_phase = 0;
                    }
                }
                break;

#if STATION_MODE
            case 11: /* 180 */
                motor_enable = 0;
                for (j = 0; j < 4; j++) {
                    target_speed[j] = 0;
                    Motor_SetSpeed(j, 0);
                }
                /*  Action_Grasp ,  */
                smooth_Settarget(0, 180, 400);
                smooth_Settarget(1, 80, 400);
                smooth_Settarget(2, 110, 400);
                /*  ~120  Phase 7,  0150  */
                {
                    uint32_t wto = g_sys_tick + 200; /* 2s */
                    while (smooth_GetCurrentAngle(0) < 120.0f
                           && g_sys_tick < wto) {
                        Delay_ms(20);
                    }
                }
                printf("Grasp done, waist=%.0f -> place\r\n",
                       (double)smooth_GetCurrentAngle(0));
                qr_phase = 7;
                qr_phase_tick = g_sys_tick;
                break;

            case 7:  /* : AprilTag     */
                {
                    uint16_t col_angle, scan_angle;
                    uint8_t  target_cls;
                    int16_t  target_tag_id;
                    uint32_t scan_timeout, align_timeout, op_timeout;
                    uint8_t  found;
                    OpenMV_Data omv_t;

                    motor_enable = 0;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        Motor_SetSpeed(j, 0);
                    }
                    /* fallback */
                    target_cls = mission_class[mission_idx];
                    /* class_id  AprilTag ID: (0)2, (2)3, (1)4 */
                    if (target_cls == 0) {
                        target_tag_id = 2;
                        col_angle = TRAILER_POS_RED;
                    } else if (target_cls == 2) {
                        target_tag_id = 3;
                        col_angle = TRAILER_POS_YELLOW;
                    } else {
                        target_tag_id = 4;
                        col_angle = TRAILER_POS_GREEN;
                    }
                    printf("Trailer: scan tag %d, fallback %d deg\r\n",
                           (int)target_tag_id, (int)col_angle);
                    /* APRILTAG */
                    OpenMV_FlushRx();  /*  AI/COLOR  */
                    OpenMV_SetMode(OMV_MODE_APRILTAG);
                    OpenMV_SetTargetTag(target_tag_id);
                    Delay_ms(400);     /*  +  */
                    OpenMV_FlushRx();  /*  COLOR $TAG */
                    /* 扫描 130~205: 覆盖红185/黄165/绿150 + 余量. 步进10°(用户要求大一点) */
                    found = 0;
                    scan_timeout = g_sys_tick + 2000;  /* 20s */
                    for (scan_angle = 130; scan_angle <= 205 && !found;
                         scan_angle += 10) {
                        int poll_i;
                        uint8_t confirm_cnt;  /* , 2 */
                        if (g_sys_tick > scan_timeout) break;
                        smooth_Settarget(0, scan_angle, 600);
                        while (smoothservo_IsBusy(0)) {
                            if (g_sys_tick > scan_timeout) break;
                            Delay_ms(10);
                        }
                        Delay_ms(500);  /* + */
                        /* , 42() */
                        {
                            uint8_t hist[12];  /* : 1= 0= */
                            uint8_t hist_idx;
                            for (poll_i = 0; poll_i < 12; poll_i++)
                                hist[poll_i] = 0;
                            hist_idx = 0;
                            confirm_cnt = 0;
                            for (poll_i = 0; poll_i < 12 && !found; poll_i++) {
                                uint8_t sum;
                                uint8_t k;
                                OpenMV_PollLine();
                                if (OpenMV_GetData(&omv_t) && omv_t.cx > 0) {
                                    printf("Trailer: saw tag %d cx=%d dist=%d "
                                           "waist=%d (want %d)\r\n",
                                           (int)omv_t.tag_id, (int)omv_t.cx,
                                           (int)omv_t.distance_cm,
                                           (int)scan_angle, (int)target_tag_id);
                                    hist[hist_idx] =
                                        (omv_t.tag_id == target_tag_id) ? 1 : 0;
                                } else {
                                    hist[hist_idx] = 0;  /* = */
                                }
                                hist_idx = (hist_idx + 1) % 12;
                                /* : 42 */
                                sum = 0;
                                for (k = 0; k < 12; k++)
                                    sum += hist[k];
                                if (sum >= 2) {
                                    found = 1;
                                    col_angle = scan_angle;
                                    printf("Trailer: MATCH! tag %d at waist=%d "
                                           "(hits=%d)\r\n",
                                           (int)target_tag_id, (int)col_angle,
                                           (int)sum);
                                }
                                Delay_ms(50);
                            }
                        }
                    }
                    /* : ,  */
                    if (found) {
                        int16_t align_cnt;
                        align_cnt = 0;
                        align_timeout = g_sys_tick + 800; /* 8s */
                        {
                            uint8_t align_lost;  /* 连续丢帧容错, 5→15: 码在视野边缘转动时易丢帧 */
                            align_lost = 0;
                            while (align_cnt < 5 && g_sys_tick <= align_timeout) {
                                float cur, delta, new_a;
                                int16_t ex;
                                OpenMV_PollLine();
                                if (!OpenMV_GetData(&omv_t)
                                    || omv_t.tag_id != target_tag_id) {
                                    align_lost++;
                                    if (align_lost >= 15) break;  /* 连续丢帧 15 次才放弃 */
                                    Delay_ms(50);
                                    continue;
                                }
                                align_lost = 0;  /* ,  */
                                ex = omv_t.cx - TRAILER_PLACE_CX;
                                if (ex < 0) ex = -ex;
                                cur = smooth_GetCurrentAngle(0);
                                delta = -(float)(omv_t.cx - TRAILER_PLACE_CX) * 0.04f;
                                if (delta > 5.0f)  delta = 5.0f;
                                if (delta < -5.0f) delta = -5.0f;
                                new_a = cur + delta;
                                if (new_a < 0.0f)   new_a = 0.0f;
                                if (new_a > 200.0f) new_a = 200.0f;
                                smooth_Settarget(0, (uint16_t)new_a, 100);
                                Delay_ms(120);
                                if (ex <= 15) align_cnt++; else align_cnt = 0;
                            }
                        }
                        col_angle = (uint16_t)smooth_GetCurrentAngle(0);
                        printf("Trailer: aligned tag %d at waist=%d\r\n",
                               (int)target_tag_id, (int)col_angle);
                    }
                    if (!found)
                        printf("Trailer: not found, blind place at %d\r\n",
                               (int)col_angle);
                    /* 用户要求: 放置一律用固定角度(稳定), 扫码只是识别过程,
                       不放码位 — 码检测/遮挡不影响放置 */
                    if (target_cls == 0)
                        col_angle = TRAILER_POS_RED;
                    else if (target_cls == 2)
                        col_angle = TRAILER_POS_YELLOW;
                    else
                        col_angle = TRAILER_POS_GREEN;
                    printf("Trailer: fixed place at %d deg\r\n", (int)col_angle);
                    /* 腰座转到放置角度(固定角度, 不能停在扫描位) */
                    smooth_Settarget(0, col_angle, 1500);
                    while (smoothservo_IsBusy(0))
                        Delay_ms(10);
                    Delay_ms(200);
                    if (!ArmSM_RequestPlace(col_angle)) {
                        printf("Trailer: ArmSM busy, force place anyway\r\n");
                    }
                    Action_PlaceOnTrailer();
                    ArmSM_NotifyComplete();
                    printf("Trailer: place done\r\n");
                    qr_phase = 8;
                }
                break;

            case 9:  /* : AI; skip_pick */
                {
                    uint16_t col_angle, deliver_angle;
                    uint8_t  target_cls, was_skip;
                    uint32_t op_to;

                    #define PHASE9_STEP_TO 300
                    motor_enable = 0;
                    for (j = 0; j < 4; j++) {
                        target_speed[j] = 0;
                        Motor_SetSpeed(j, 0);
                    }
                    was_skip = qr_skip_pick;
                    qr_skip_pick = 0;  /*  */

                    if (!was_skip) {
                    /* === :  === */
                    /* : () (190)(180)(170) */
                    target_cls = mission_class[MISSION_COUNT - 1
                                               - mission_idx];
                    if (target_cls == 0)
                        col_angle = TRAILER_POS_RED;
                    else if (target_cls == 2)
                        col_angle = TRAILER_POS_YELLOW;
                    else
                        col_angle = TRAILER_POS_GREEN;
                    printf("Delivery: pick col %d deg (cls %d)\r\n",
                           (int)col_angle, (int)target_cls);
                    /*  () */
                    if (!ArmSM_RequestGrasp()) {
                        printf("Delivery: ArmSM busy, force pick\r\n");
                    }
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(0, col_angle, 1500);
                    while (smoothservo_IsBusy(0) && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(300);
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(1, act_angle[ACT_APPROACH_SHOULDER], 1000);
                    smooth_Settarget(2, act_angle[ACT_APPROACH_ELBOW], 1000);
                    while (smoothservo_AnyBusy() && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(500);
                    /* Bug#8 fix: smooth_Settarget  Servo_SetAngle */
                    smooth_Settarget(3, act_angle[ACT_GRIPPER_CLOSE], 500);
                    while (smoothservo_IsBusy(3))
                        Delay_ms(10);
                    Delay_ms(300);
                    /*  */
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(2, ARM_HOME_ELBOW, 1500);
                    Delay_ms(300);
                    smooth_Settarget(1, ARM_HOME_SHOULDER, 1500);
                    Delay_ms(300);
                    smooth_Settarget(0, 90, 1500);
                    while (smoothservo_AnyBusy() && g_sys_tick < op_to)
                        Delay_ms(10);
                    ArmSM_NotifyComplete();  /*  */
                    Delay_ms(500);
                    } else {
                    /* === : Phase6,  === */
                    printf("Delivery: skip pick, already holding"
                           " (color=%d)\r\n", (int)qr_target_color);
                    /*  () 腰座转到交付写死角度 */
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(2, ARM_HOME_ELBOW, 1500);
                    Delay_ms(300);
                    smooth_Settarget(1, ARM_HOME_SHOULDER, 1500);
                    Delay_ms(300);
                    smooth_Settarget(0, (uint16_t)deliver_waist[qr_target_color], 1500);
                    while (smoothservo_AnyBusy() && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(500);
                    /* ,  */
                    }

                    /* 交付角度 = 按颜色写死 (红90/绿100/黄80) */
                    deliver_angle = (uint16_t)deliver_waist[qr_target_color];
                    printf("Delivery: place color %d at %d deg\r\n",
                           (int)qr_target_color, (int)deliver_angle);
                    /*  () */
                    if (!ArmSM_RequestPlace(deliver_angle)) {
                        printf("Delivery: ArmSM busy, force place\r\n");
                    }
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(0, deliver_angle, 1500);
                    while (smoothservo_IsBusy(0) && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(300);
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(1, 110, 1000);  /* shoulder deliver */
                    smooth_Settarget(2, 100, 1000);  /* elbow deliver */
                    while (smoothservo_AnyBusy() && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(500);
                    /* Bug#8 fix: smooth_Settarget  Servo_SetAngle */
                    smooth_Settarget(3, act_angle[ACT_GRIPPER_OPEN], 500);
                    while (smoothservo_IsBusy(3))
                        Delay_ms(10);
                    Delay_ms(300);
                    /* 开爪放物体后: 先抬臂离开交付位, 再关爪收尾 */
                    op_to = g_sys_tick + PHASE9_STEP_TO;
                    smooth_Settarget(1, 30, 1500);   /* 大臂抬起 */
                    smooth_Settarget(2, 170, 1500);  /* 小臂抬起 */
                    while (smoothservo_AnyBusy() && g_sys_tick < op_to)
                        Delay_ms(10);
                    Delay_ms(300);
                    smooth_Settarget(3, act_angle[ACT_GRIPPER_CLOSE], 500);
                    while (smoothservo_IsBusy(3))
                        Delay_ms(10);
                    Delay_ms(300);
                    ArmSM_NotifyComplete();  /*  */
                    printf("Delivery: done\r\n");
                    qr_phase = 8;
                }
                break;

            case 8:  /*  */
                mission_idx++;
                printf("Case8: m=%d st=%d dlv=%d\r\n",
                       (int)mission_idx, (int)station_idx,
                       (int)g_delivery_mode);
                if (mission_idx < MISSION_COUNT) {
                    /* : = =, delivery */
                    if ((station_idx & 1) == 0 && !g_delivery_mode) {
                        /* (mission_idx=2, class=1): AI,
                           COLOR, AI.
                           TODO: >70%, ,
                           /AI */
                        if (mission_idx == 2 && mission_class[2] == 1) {
                            /* 绿色: 物体39-54°, 识别中心 50°.
                               用 AI 识别(认 green_circle 形状) —
                               COLOR,2 阈值与黄色重叠会误认黄物体 */
                            smooth_Settarget(0, 50, 800);
                            smooth_Settarget(1, 50, 600);
                            smooth_Settarget(2, 160, 600);
                            while (smoothservo_IsBusy(0))
                                Delay_ms(20);
                            Delay_ms(300);
                            qr_target_color = 2;
                            OpenMV_ClearAIError();
                            OpenMV_SendCmd("MODE,AI");
                            OpenMV_FlushRx();
                            qr_scan_started = 0;
                            qr_phase_tick = g_sys_tick;
                            phase1_scan_step = 0;
                            phase1_scan_tick = 0;
                            qr_phase2_arm_done = 0;  /* 0=识别阶段 */
                            id_hit_cnt = 0;
                            qr_phase = 1;
                            printf("Station %d pickup: green -> AI identify+QA\r\n",
                                   (int)station_idx);
                        } else {
                            /* 红/黄: COLOR 识别 -> case1 (黄物体>90°, 识别中心110°) */
                            char cbuf[16];
                            smooth_Settarget(0,
                                (uint16_t)((mission_class[mission_idx] == 2)
                                    ? 110 : 90), 800);
                            smooth_Settarget(1, 50, 600);
                            smooth_Settarget(2, 160, 600);
                            while (smoothservo_IsBusy(0))
                                Delay_ms(20);
                            Delay_ms(300);
                            qr_target_color = (uint8_t)(mission_class[mission_idx] + 1);
                            if (qr_target_color < 1) qr_target_color = 1;
                            if (qr_target_color > 3) qr_target_color = 3;
                            sprintf(cbuf, "MODE,COLOR,%d", (int)qr_target_color);
                            OpenMV_FlushRx();
                            OpenMV_SendCmd(cbuf);
                            qr_scan_started = 0;
                            qr_phase_tick = g_sys_tick;
                            phase1_scan_step = 0;
                            phase1_scan_tick = 0;
                            qr_phase2_arm_done = 0;  /* 0=识别阶段 */
                            id_hit_cnt = 0;
                            qr_phase = 1;
                            printf("Station %d pickup: next obj %d/%d color %d\r\n",
                                   (int)station_idx, (int)mission_idx,
                                   MISSION_COUNT, (int)qr_target_color);
                        }
                    } else {
                        /* Station 1: 下一件 -> 90°识别(COLOR) -> 170°定位 */
                        {
                            char cbuf[16];
                            qr_target_color = (uint8_t)(mission_class[mission_idx] + 1);
                            if (qr_target_color < 1) qr_target_color = 1;
                            if (qr_target_color > 3) qr_target_color = 3;
                            sprintf(cbuf, "MODE,COLOR,%d", (int)qr_target_color);
                            OpenMV_SendCmd(cbuf);
                        }
                        OpenMV_FlushRx();
                        /* 识别姿势: 复位(大臂0/小臂120) + 腰座按颜色写死角 */
                        smooth_Settarget(0, (uint16_t)identify_waist[qr_target_color], 600);
                        smooth_Settarget(1, ARM_HOME_SHOULDER, 600);
                        smooth_Settarget(2, ARM_HOME_ELBOW,    600);
                        qr_scan_started = 0;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = 0;
                        qr_phase2_arm_done = 0;  /* 识别阶段 */
                        id_hit_cnt = 0;
                        qr_phase = 1;
                        printf("Station %d: next obj %d/%d, identify color %d at %d\r\n",
                               (int)station_idx, (int)mission_idx,
                               MISSION_COUNT, (int)qr_target_color,
                               (int)identify_waist[qr_target_color]);
                    }
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                    dbg_tick     = 0;
                    phase1_scan_step = 0;
                    phase1_scan_tick = 0;
                } else {
                    /* 3,  */
                    g_delivery_mode = 0;  /* delivery */
                    smooth_Settarget(3, act_angle[ACT_GRIPPER_CLOSE], 500);
                    smooth_Settarget(2, ARM_HOME_ELBOW, 1500);
                    smooth_Settarget(1, ARM_HOME_SHOULDER, 1500);
                    smooth_Settarget(0, ARM_HOME_WAIST, 1500);
                    while (smoothservo_AnyBusy())
                        Delay_ms(20);
                    Delay_ms(300);
                    station_idx++;
                    if (station_idx < total_stations) {
                        /* QR */
                        mission_idx = 0;
                        OpenMV_SendCmd("MODE,QRCODE");
                        qr_cooldown_end = g_sys_tick + 500;
                        LineFollow_Reset(&line_follow);
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                        target_speed[0] = 180;
                        target_speed[1] = 180;
                        target_speed[2] = 180;
                        target_speed[3] = 170;
                        qr_phase = 0;
                        qr_scan_started = 0;
                        nodata_start = 0;
                        scan_step    = 0;
                        scan_tick    = 0;
                        dbg_tick     = 0;
                        printf("Station %d done, heading to station %d\r\n",
                               (int)(station_idx - 1), (int)station_idx);
                    } else {
                        /*  */
                        mission_done = 1;
                        flow_stage = 4;     /* LED 常亮 = 全部完成 */
                        qr_phase = 0;
                        printf("ALL STATIONS COMPLETE!\r\n");
                    }
                }
                break;
#endif

#if !STATION_MODE
            case 4:  /* : QRAICOLOR
                        : qr_phase2_arm_done: 0=QR, 1=AI, 2=AI */
                {
                    OpenMV_QRData qr;

                    /* QR,  */
                    if (g_sys_tick < qr_cooldown_end) {
                        OpenMV_PollLine();
                        (void)OpenMV_GetQRData(&qr);
                        break;
                    }

                    /* --- 0: QR --- */
                    if (qr_phase2_arm_done == 0) {
                        OpenMV_PollLine();
                        if (OpenMV_GetQRData(&qr)) {
                            int jj;
                            /* QR      AI */
                            printf("Trailer: QR %s -> AI scan\r\n", qr.text);
                            motor_enable = 0;
                            work_mode = MODE_MANUAL;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                            Delay_ms(300);

                            /*  */
                            smooth_Settarget(0, act_angle[ACT_APPROACH_WAIST], 1000);
                            smooth_Settarget(1, act_angle[ACT_PLACE_SHOULDER], 1000);
                            smooth_Settarget(2, act_angle[ACT_APPROACH_ELBOW], 1200);
                            Delay_ms(1500);

                            /*  AI  */
                            OpenMV_ClearAIError();
                            OpenMV_SendCmd("MODE,AI");
                            qr_scan_started = 0;
                            qr_phase_tick = g_sys_tick;
                            phase1_scan_step = 0;
                            phase1_scan_tick = 0;
                            qr_phase2_arm_done = 1;
                            printf("Trailer: AI scan, target cls=%d\r\n",
                                   (int)mission_class[mission_idx]);
                        }
                        /* 60s    */
                        if (g_sys_tick - qr_place_phase_start > 6000) {
                            int jj;
                            printf("Trailer: timeout (60s), abort\r\n");
                            motor_enable = 0;
                            work_mode = MODE_MANUAL;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                            qr_phase2_arm_done = 0;
                            OpenMV_SendCmd("MODE,QRCODE");
                            qr_cooldown_end = g_sys_tick + 500;
                            qr_phase = 0;
                        }
                        break;
                    }

                    /* --- 1: AI --- */
                    if (qr_phase2_arm_done == 1) {
                        OpenMV_PollLine();

                        if (OpenMV_IsModelError()) {
                            int jj;
                            printf("Trailer: AI model error, abort\r\n");
                            OpenMV_SendCmd("MODE,QRCODE");
                            qr_cooldown_end = g_sys_tick + 500;
                            qr_phase2_arm_done = 0;
                            LineFollow_Reset(&line_follow);
                            work_mode = MODE_LINE_FOLLOW;
                            motor_enable = 1;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                            target_speed[0] = 180; target_speed[1] = 180;
                            target_speed[2] = 180; target_speed[3] = 170;
                            qr_phase = 0;
                            break;
                        }
                        if (g_sys_tick - qr_phase_tick > 300
                            && !OpenMV_IsAlive(1200)) {
                            int jj;
                            printf("Trailer: OpenMV crash, abort\r\n");
                            OpenMV_SendCmd("MODE,QRCODE");
                            qr_cooldown_end = g_sys_tick + 500;
                            qr_phase2_arm_done = 0;
                            LineFollow_Reset(&line_follow);
                            work_mode = MODE_LINE_FOLLOW;
                            motor_enable = 1;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                            target_speed[0] = 180; target_speed[1] = 180;
                            target_speed[2] = 180; target_speed[3] = 170;
                            qr_phase = 0;
                            break;
                        }
                        if (!OpenMV_IsAIReady()
                            && g_sys_tick - qr_phase_tick < 6000) {
                            break;
                        }
                        if (!OpenMV_IsAIReady()) {
                            int jj;
                            printf("Trailer: AI wait timeout, abort\r\n");
                            OpenMV_SendCmd("MODE,QRCODE");
                            qr_cooldown_end = g_sys_tick + 500;
                            qr_phase2_arm_done = 0;
                            LineFollow_Reset(&line_follow);
                            work_mode = MODE_LINE_FOLLOW;
                            motor_enable = 1;
                            for (jj = 0; jj < 4; jj++) {
                                target_speed[jj] = 0;
                                Motor_SetSpeed(jj, 0);
                            }
                            target_speed[0] = 180; target_speed[1] = 180;
                            target_speed[2] = 180; target_speed[3] = 170;
                            qr_phase = 0;
                            break;
                        }
                        /* , 1s */
                        if (g_sys_tick - qr_phase_tick < 100) {
                            break;
                        }
                        qr_waist_center = smooth_GetCurrentAngle(0);
                        qr_scan_started = 1;
                        qr_phase_tick = g_sys_tick;
                        phase1_scan_step = 0;
                        phase1_scan_tick = (uint16_t)(g_sys_tick - 31);
                        qr_phase2_arm_done = 2;
                        printf("Trailer: scan start, center=%.0f\r\n",
                               (double)qr_waist_center);
                        break;
                    }

                    /* --- 2: AI (Phase1) --- */
                    {
                        OpenMV_PollLine();
                        if (!smoothservo_IsBusy(0)
                            && (uint16_t)(g_sys_tick - phase1_scan_tick) > 60) {
                            /*  AI */
                            {
                                OpenMV_CLSData dbg_cls;
                                if (OpenMV_GetCLSData(&dbg_cls)) {
                                    printf("Trailer AI: cls=%d conf=%d%% "
                                           "[%d,%d,%d] "
                                           "waist=%.0f need cls=%d conf>=%d"
                                           "\r\n",
                                           (int)dbg_cls.class_id,
                                           (int)dbg_cls.confidence,
                                           (int)dbg_cls.score[0],
                                           (int)dbg_cls.score[1],
                                           (int)dbg_cls.score[2],
                                           (double)smooth_GetCurrentAngle(0),
                                           (int)mission_class[mission_idx],
                                           (int)color_conf_min[
                                               mission_class[mission_idx]]);
                                    cls = dbg_cls;
                                } else {
                                    cls.class_id = -1;
                                }
                            }
                            if (cls.class_id == mission_class[mission_idx]
                                && cls.confidence >=
                                    color_conf_min[cls.class_id]) {
                                printf("Trailer AI HIT: class=%d conf=%d%% "
                                       "waist=%.0f\r\n",
                                       (int)cls.class_id,
                                       (int)cls.confidence,
                                       (double)smooth_GetCurrentAngle(0));
                                smooth_Settarget(0,
                                    (uint16_t)smooth_GetCurrentAngle(0), 40);
                                qr_target_color =
                                    (uint8_t)(cls.class_id + 1);
                                if (qr_target_color < 1) qr_target_color = 1;
                                if (qr_target_color > 3) qr_target_color = 3;
                                {
                                    char cbuf[16];
                                    sprintf(cbuf, "MODE,COLOR,%d",
                                            (int)qr_target_color);
                                    OpenMV_SendCmd(cbuf);
                                }
                                OpenMV_FlushRx();
                                Delay_ms(300);
                                qr_phase2_arm_done = 0;
                                qr_phase = 5;
                                qr_phase_tick = g_sys_tick;
                                qr_aligned_cnt = 0;
                                qa_window = 0;
                                break;
                            }
                            /* : 20, 5, 16 */
                            {
                                float target;
                                int step_n, tri_idx, offset;
                                step_n = phase1_scan_step;
                                tri_idx = (step_n + 1) % 16;
                                if (tri_idx == 0)
                                    offset = 0;
                                else if (tri_idx <= 4)
                                    offset = tri_idx * 5;
                                else if (tri_idx <= 8)
                                    offset = (8 - tri_idx) * 5;
                                else if (tri_idx <= 12)
                                    offset = (8 - tri_idx) * 5;
                                else
                                    offset = (tri_idx - 16) * 5;
                                target = qr_waist_center + (float)offset;
                                phase1_scan_step++;
                                if (phase1_scan_step >= 16) {
                                    phase1_scan_step = 0;
                                    printf("Trailer: scan loop\r\n");
                                }
                                if (target < 0.0f) target = 0.0f;
                                if (target > 270.0f) target = 270.0f;
                                printf("Trailer: step=%d -> %.0f\r\n",
                                       (int)phase1_scan_step,
                                       (double)target);
                                smooth_Settarget(0, (uint16_t)target, 300);
                                phase1_scan_tick = (uint16_t)g_sys_tick;
                            }
                        }
                        /* 15s    */
                        if (g_sys_tick - qr_phase_tick > 1500) {
                            if (mission_class[mission_idx] == 0
                                || mission_class[mission_idx] == 1) {
                                printf("Trailer: AI timeout cls=%d, "
                                       "fallback COLOR\r\n",
                                       (int)mission_class[mission_idx]);
                                if (mission_class[mission_idx] == 0)
                                    qr_target_color = 1;
                                else
                                    qr_target_color = 2;
                                qr_waist_center = smooth_GetCurrentAngle(0);
                                qr_aligned_cnt = 0;
                                qa_window = 0;
                                {
                                    char cbuf[16];
                                    sprintf(cbuf, "MODE,COLOR,%d",
                                            (int)qr_target_color);
                                    OpenMV_SendCmd(cbuf);
                                }
                                OpenMV_FlushRx();
                                Delay_ms(200);
                                qr_phase2_arm_done = 0;
                                qr_phase = 5;
                                qr_phase_tick = g_sys_tick;
                            } else {
                                int jj;
                                printf("Trailer: AI scan timeout, abort\r\n");
                                qr_phase2_arm_done = 0;
                                qr_scan_started = 0;
                                OpenMV_SendCmd("MODE,QRCODE");
                                qr_cooldown_end = g_sys_tick + 500;
                                LineFollow_Reset(&line_follow);
                                work_mode = MODE_LINE_FOLLOW;
                                motor_enable = 1;
                                for (jj = 0; jj < 4; jj++) {
                                    target_speed[jj] = 0;
                                    Motor_SetSpeed(jj, 0);
                                }
                                target_speed[0] = 180;
                                target_speed[1] = 180;
                                target_speed[2] = 180;
                                target_speed[3] = 170;
                                qr_phase = 0;
                            }
                        }
                    }
                }
                break;

            case 5:  /* : COLOR (Phase10) */
                {
                    int jj;
                    OpenMV_PollLine();
                    if (OpenMV_GetData(&omv) && omv.distance_cm > 0) {
                        int16_t error_x, abs_x, tgt_cx;
                        tgt_cx = color_cx[qr_target_color - 1];
                        error_x = omv.cx - tgt_cx;
                        abs_x = (error_x < 0) ? -error_x : error_x;
                        /*  */
                        {
                            float cur_a, delta, new_a;
                            cur_a = smooth_GetCurrentAngle(0);
                            delta = -(float)error_x * 0.05f;
                            if (delta > 8.0f)  delta = 8.0f;
                            if (delta < -8.0f) delta = -8.0f;
                            new_a = cur_a + delta;
                            if (new_a < 0.0f)   new_a = 0.0f;
                            if (new_a > 180.0f) new_a = 180.0f;
                            smooth_Settarget(0, (uint16_t)new_a, 150);
                        }
                        /* : 53 */
                        {
                            uint8_t bits, pass_count;
                            qa_window = (uint8_t)((qa_window << 1) & 0x1F);
                            if (abs_x <= 20) {
                                qa_window |= 1;
                            }
                            bits = qa_window;
                            pass_count = 0;
                            while (bits) { pass_count++; bits &= bits - 1; }
                            qr_aligned_cnt = pass_count;
                            if (pass_count >= 3) {
                                printf("Trailer QA: cx=%d ok "
                                       "(window=0x%02X), grasp\r\n",
                                       (int)omv.cx, (unsigned)qa_window);
                                motor_enable = 0;
                                work_mode = MODE_MANUAL;
                                for (jj = 0; jj < 4; jj++) {
                                    target_speed[jj] = 0;
                                    Motor_SetSpeed(jj, 0);
                                }
                                qr_phase = 6;
                                qr_phase_tick = g_sys_tick;
                            }
                        }
                        if ((uint16_t)(g_sys_tick - dbg_tick) >= 50) {
                            printf("QA: cx=%d dist=%d cnt=%d\r\n",
                                   (int)omv.cx, (int)omv.distance_cm,
                                   (int)qr_aligned_cnt);
                            dbg_tick = (uint16_t)g_sys_tick;
                        }
                    }
                    /* 5s:  */
                    if (g_sys_tick - qr_phase_tick > 500) {
                        printf("Trailer QA: timeout, grasp anyway\r\n");
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (jj = 0; jj < 4; jj++) {
                            target_speed[jj] = 0;
                            Motor_SetSpeed(jj, 0);
                        }
                        qr_phase = 6;
                        qr_phase_tick = g_sys_tick;
                    }
                }
                break;

            case 6:  /* 0=, 1=Phase9 */
                {
                    int jj;
                    uint16_t saved_waist;
                    motor_enable = 0;
                    for (jj = 0; jj < 4; jj++) {
                        target_speed[jj] = 0;
                        Motor_SetSpeed(jj, 0);
                    }
                    Delay_ms(200);
                    saved_waist = act_angle[ACT_APPROACH_WAIST];
                    act_angle[ACT_APPROACH_WAIST] =
                        (uint16_t)smooth_GetCurrentAngle(0);
                    if ((station_idx & 1) == 0) {
                        /* 0:  */
                        printf("Trailer: PLACE onto trailer"
                               " (waist=%d)\r\n",
                               (int)smooth_GetCurrentAngle(0));
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
                        qr_cooldown_end = g_sys_tick + 500;
                        LineFollow_Reset(&line_follow);
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                        target_speed[0] = 180; target_speed[1] = 180;
                        target_speed[2] = 180; target_speed[3] = 170;
                        qr_is_place = 0;
                        /*  */
                        mission_idx++;
                        if (mission_idx >= MISSION_COUNT) {
                            station_idx++;
                            if (station_idx >= total_stations) {
                                mission_done = 1;
                                printf("ALL STATIONS DONE: %d objects\r\n",
                                       MISSION_COUNT);
                            } else {
                                mission_idx = 0;
                                printf("Station %d done, -> station %d\r\n",
                                       (int)(station_idx - 1),
                                       (int)station_idx);
                            }
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
                        printf("Trailer place done, resume line follow\r\n");
                    } else {
                        /* 1:   Phase9() */
                        printf("Trailer: GRASP from trailer"
                               " (waist=%d)\r\n",
                               (int)smooth_GetCurrentAngle(0));
                        if (ArmSM_RequestGrasp()) {
                            g_skip_waist_home = 1;
                            Action_Grasp();
                            g_skip_waist_home = 0;
                            ArmSM_NotifyComplete();
                        } else {
                            printf("Trailer: arm busy, skip grasp\r\n");
                        }
                        act_angle[ACT_APPROACH_WAIST] = saved_waist;
                        OpenMV_FlushRx();
                        qr_skip_pick = 1;  /* Phase9  */
                        qr_phase = 9;
                        qr_phase_tick = g_sys_tick;
                        printf("Trailer grasp done, -> delivery place\r\n");
                    }
                }
                break;
#endif /* !STATION_MODE */

            default:
                /* Bugfix H4: phase,  */
                {
                    int di;
                    printf("AI: invalid phase %d, emergency cleanup\r\n", (int)qr_phase);
                    motor_enable = 0;
                    for (di = 0; di < 4; di++) {
                        target_speed[di] = 0;
                        Motor_SetSpeed(di, 0);
                    }
                    qr_scan_started = 0;
                    nodata_start = 0;
                    scan_step    = 0;
                    scan_tick    = 0;
                    phase1_scan_step = 0;
                    phase1_scan_tick = 0;

                    qr_phase2_arm_done = 0;
                    qr_phase = 0;
                }
                break;
            }
            }   /* end: if (!dock_tune_on) 冻结AI */
        }
#endif /* !TEST_MODE */

        /* LED: 阶段指示 (无串口也能看进度)
           flow_stage: 1=巡线(慢闪500ms) 2=对接(快闪100ms) 3=工位(中闪250ms) 4=全部完成(常亮) */
        {
            static uint32_t led_next = 0;
            uint32_t period;
            uint32_t now;
            now = g_sys_tick;
            if (flow_stage == 4) {
                GPIO_ResetBits(GPIOF, GPIO_Pin_9);   /* 常亮 = 全部完成 */
            } else {
                if (flow_stage == 2) {
                    period = 10;                     /* 100ms 快闪 = 对接中 */
                } else if (flow_stage == 3) {
                    period = 25;                     /* 250ms 中闪 = 工位作业 */
                } else {
                    period = 50;                     /* 500ms 慢闪 = 巡线等待 */
                }
                if (now >= led_next) {
                    led_next = now + period;
                    GPIOF->ODR ^= GPIO_Pin_9;
                }
            }
        }

        /* 串口心跳: 巡线阶段(flow_stage=1)每2s打一行 — 验证链路干净+确认活着.
           若这行也断字/串行, 是终端/串口工具问题(开晚/缓冲满), 换工具或提前开端口 */
        {
            static uint32_t beacon_tick = 0;
            if (flow_stage == 1 && g_sys_tick - beacon_tick >= 200) {
                beacon_tick = g_sys_tick;
                printf("TICK: uptime=%lu s\r\n",
                       (unsigned long)(g_sys_tick / 100));
            }
        }

        /* 巡线调试: lfdbg 命令开关, 每100ms打印 传感器/编码器上限/目标速度/实际速度/PID输出 */
        {
            static uint32_t lfd_tick = 0;
            if (lf_dbg_enable && g_sys_tick - lfd_tick >= 10) {
                bool ls[5];
                int li;
                lfd_tick = g_sys_tick;
                LineSensor_Read(ls);
                printf("LF: wm=%d en=%d ls[%d%d%d%d%d] em[%d%d%d%d]\r\n",
                       (int)work_mode, (int)motor_enable,
                       (int)ls[0], (int)ls[1], (int)ls[2], (int)ls[3],
                       (int)ls[4],
                       (int)encoder_max[0], (int)encoder_max[1],
                       (int)encoder_max[2], (int)encoder_max[3]);
                printf("LF: spd=%d,%d,%d,%d real=%d,%d,%d,%d"
                       " out=%d,%d,%d,%d\r\n",
                       (int)target_speed[0], (int)target_speed[1],
                       (int)target_speed[2], (int)target_speed[3],
                       (int)(Encoder_getspeed(0) * encoder_dir[0]),
                       (int)(Encoder_getspeed(1) * encoder_dir[1]),
                       (int)(Encoder_getspeed(2) * encoder_dir[2]),
                       (int)(Encoder_getspeed(3) * encoder_dir[3]),
                       (int)last_pid_output[0], (int)last_pid_output[1],
                       (int)last_pid_output[2], (int)last_pid_output[3]);
            }
        }

        /* CSV telemetry: LLM PID Tuner (stm32f407_openmv profile) */
        {
            static uint32_t last_send_tick = 0;
            if (telem_enable && (g_sys_tick - last_send_tick >= 5)) {
                last_send_tick = g_sys_tick;   /* 节流: 每50ms一发, 防刷屏挤爆串口 */
                sendVisualServoTelemetry();   /* 视觉伺服双PID遥测 */
            }
        }

        OpenMV_PollLine();

#if 0  /* ==== DOCK STATE MACHINE DISABLED: go straight to grasp/place ==== */
        /* ===== 从车对接状态机 (十字路口 -> 右拐脱线 -> 倒车AprilTag对接) ===== */
        {
            /* dock_state/dock_tick/dock_lost/dock_offline_cnt 全局 (dockstat 可读) */
            OpenMV_Data omv_d;   /* C90: 块首声明 */

            if (trailer_dock_flag && dock_state == 0) {
                /* 触发: ISR 检测到十字路口 → 停车原地右转90° 后对接 */
                printf("DOCK: crossroad detected, stop & pivot\r\n");
                dock_state = 1;
                dock_tick = g_sys_tick;  /* state1 停车/旋转计时基准 */
                dock_lost = 0;
                dock_offline_cnt = 0;
                dock_tune_on = 0;       /* 正式任务对接恢复挂钩 */
                dock_pv_phase = 0;      /* 每次触发重新: 停车→旋转 */
                dock_pv_dropped = 0;
                flow_stage = 2;         /* LED 快闪 = 对接中 */
            }

            if (dock_test_flag) {
                /* 调试: 车已对准从车, 直接初始化并进倒车对接 (跳过右拐/找码) */
                static uint8_t test_once = 0;
                dock_test_flag = 0;
                if (!test_once) {
                    test_once = 1;
                    dock_tune_on = 0;   /* 单次测试: 正常挂钩 */
                    printf("DOCK: test mode, init & reverse dock\r\n");
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        Motor_SetSpeed(i, 0);
                    }
                    OpenMV_FlushRx();
                    OpenMV_SetMode(OMV_MODE_APRILTAG);
                    OpenMV_SetTargetTag(1);
                    smooth_Settarget(0, 180, 1000);   /* 腰座转车尾看从车 */
                    VisualServo_Reset(&visual_servo);   /* 清积分残留 */
                    dock_state = 3;   /* 直接进倒车阶段 */
                    dock_tick = g_sys_tick;
                    dock_lost = 0;
                }
                /* 腰座到位后 test_once 复位, 下次可再用 */
                if (!smoothservo_IsBusy(0) && dock_state == 3 && test_once) {
                    test_once = 0;
                }
            }

            if (dock_tune_flag) {
                /* 调参模式: 自动循环倒车对接 (到38cm->前进离开->再倒车), 不挂钩
                   再发一次 dock_tune = 退出, 恢复 AI 自动任务 */
                dock_tune_flag = 0;
                if (dock_tune_on) {
                    /* 退出调参模式 */
                    dock_tune_on = 0;
                    dock_tune_once = 0;
                    dock_state = 0;
                    dock_lost = 0;
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        Motor_SetSpeed(i, 0);
                    }
                    printf("DOCK: tune mode OFF, AI resumed\r\n");
                } else if (!dock_tune_once) {
                    dock_tune_once = 1;
                    dock_tune_on = 1;
                    printf("DOCK: tune mode, auto-loop reverse dock\r\n");
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        Motor_SetSpeed(i, 0);
                    }
                    OpenMV_FlushRx();
                    OpenMV_SetMode(OMV_MODE_APRILTAG);
                    OpenMV_SetTargetTag(1);
                    smooth_Settarget(0, 180, 1000);   /* 腰座转车尾看从车 */
                    VisualServo_Reset(&visual_servo);   /* 清积分残留 */
                    dock_state = 3;   /* 直接进倒车阶段 */
                    dock_tick = g_sys_tick;
                    dock_lost = 0;
                }
                /* 腰座到位后 tune_once 复位, 下次可再用 */
                if (!smoothservo_IsBusy(0) && dock_state == 3 && dock_tune_once) {
                    dock_tune_once = 0;
                }
            }

            if (dock_state == 1) {
                /* 十字路口原地右转90°: 停车 → 原地差速右转(左+150/右-150) → 停 → 转腰座对接.
                   停止条件(三选一, 先到先停):
                     1) 左前编码器累计位移 ≥ DOCK_PIVOT_90_CNT (90°估算值)
                     2) 中心C+最左L1+最右R1 三个传感器"先灭后亮" = 阵列对齐竖线 = 90° (用户实测判据)
                     3) DOCK_PIVOT_TO 超时 → 放弃对接恢复巡线 */
                bool ls[5];
                uint8_t li, online = 0;
                int32_t enc0;      /* 左前编码器带符号值 (正=左轮前进=右转, 负=左轮后退=左转!) */
                int32_t enc_abs;
                LineSensor_Read(ls);
                for (li = 0; li < 5; li++)
                    if (ls[li]) online++;

                if (dock_pv_phase == 0) {
                    /* 停车 + 清速度PID残留: 巡线时4轮PID输出全在前进方向,
                       不清的话右轮接到-150目标后输出摆不过去, 车变成"偏右前进"而不是原地转 */
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        Motor_SetSpeed(i, 0);
                        PID_Reset(&pid_motor[i]);
                    }
                    Delay_ms(200);
                    Encoder_ClearPosition(0);   /* 左前编码器清零, 测转角 */
                    dock_pv_dropped = 0;
                    dock_tick = g_sys_tick;
                    dock_pv_phase = 1;
                    printf("DOCK: stop at crossroad, pivot right 90 deg\r\n");
                } else {
                    /* 原地右转: 开环直驱 (motor_enable=0, 不经PID不依赖编码器).
                       Motor_SetSpeed 要的是"方向修正后的out" = motor_dir×目标:
                       右前/右后(电机1/3, dir=-1)的IN1才是倒转方向, 直接传-200会变成前进(实测直行!) */
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    target_speed[0] = DOCK_PIVOT_SPD;
                    target_speed[1] = -DOCK_PIVOT_SPD;
                    target_speed[2] = DOCK_PIVOT_SPD;
                    target_speed[3] = -DOCK_PIVOT_SPD;
                    Motor_SetSpeed(0, (int16_t)(motor_dir[0] * DOCK_PIVOT_SPD));
                    Motor_SetSpeed(1, (int16_t)(motor_dir[1] * (-DOCK_PIVOT_SPD)));
                    Motor_SetSpeed(2, (int16_t)(motor_dir[2] * DOCK_PIVOT_SPD));
                    Motor_SetSpeed(3, (int16_t)(motor_dir[3] * (-DOCK_PIVOT_SPD)));
                    /* "深离开"0°判据: 在线≤2路 (起步时右端ls[0]抖动只会到4-5路, 不会误触发) */
                    if (online <= 2) dock_pv_dropped = 1;
                    enc0 = Encoder_GetPosition(0);
                    enc_abs = (enc0 < 0) ? -enc0 : enc0;
                    /* 周期调试: 每200ms打印旋转过程 (串口直接看问题, 无需命令) */
                    {
                        static uint32_t pdbg_tick = 0;
                        if (g_sys_tick - pdbg_tick >= 20) {
                            pdbg_tick = g_sys_tick;
                            printf("PVT: enc=%ld(abs %ld) online=%d"
                                   " ls[%d %d %d %d %d] drop=%d"
                                   " to=%lu spd=%d,%d,%d,%d\r\n",
                                   (long)enc0, (long)enc_abs,
                                   (int)online,
                                   (int)ls[0], (int)ls[1], (int)ls[2],
                                   (int)ls[3], (int)ls[4],
                                   (int)dock_pv_dropped,
                                   (unsigned long)(DOCK_PIVOT_TO
                                                    - (g_sys_tick - dock_tick)),
                                   (int)target_speed[0], (int)target_speed[1],
                                   (int)target_speed[2], (int)target_speed[3]);
                        }
                    }
                    /* 90° 到位: "先灭后亮"且在线≥4路 = 阵列对齐竖线 (实测右端ls[0]不亮,
                       不能用 中心+两端 判据); 编码器乱转不判停; 最少转 DOCK_PIVOT_MIN 后才允许判 */
                    if (g_sys_tick - dock_tick >= DOCK_PIVOT_MIN
                        && dock_pv_dropped && online >= 4) {
                        /* 停 → 转腰座对接 */
                        printf("DOCK: pivot done (enc=%ld online=%d)\r\n",
                               (long)enc_abs, (int)online);
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            Motor_SetSpeed(i, 0);
                        }
                        Delay_ms(200);
                        dock_pv_phase = 0;
                        dock_state = 2;
                        dock_tick = g_sys_tick;
                    } else if (g_sys_tick - dock_tick >= DOCK_PIVOT_CAP) {
                        /* 时间兜底: 已转足够角度, 停 (现场按实际角度调 DOCK_PIVOT_CAP) */
                        printf("DOCK: pivot cap (enc=%ld online=%d)\r\n",
                               (long)enc_abs, (int)online);
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            Motor_SetSpeed(i, 0);
                        }
                        Delay_ms(200);
                        dock_pv_phase = 0;
                        dock_state = 2;
                        dock_tick = g_sys_tick;
                    } else if (g_sys_tick - dock_tick > DOCK_PIVOT_TO) {
                        /* 安全超时: 放弃对接, 恢复巡线, 下个路口重试 */
                        printf("DOCK: pivot timeout, reset dock\r\n");
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            Motor_SetSpeed(i, 0);
                        }
                        dock_pv_phase = 0;
                        dock_state = 0;
                        trailer_dock_flag = 0;
                        OpenMV_FlushRx();
                        OpenMV_SendCmd("MODE,QRCODE");
                        LineFollow_Reset(&line_follow);
                        work_mode = MODE_LINE_FOLLOW;
                        motor_enable = 1;
                        target_speed[0] = 180; target_speed[1] = 180;
                        target_speed[2] = 180; target_speed[3] = 170;
                    }
                }
            } else if (dock_state == 2) {
                /* 脱线停车: 腰座转180°看车尾, 切APRILTAG找ID=1 */
                if (!dock_st2_init) {
                    dock_st2_init = 1;
                    dock_st2_start = g_sys_tick;   /* 30s 总超时起点 */
                    printf("DOCK: waist to 180, find tag1\r\n");
                    OpenMV_FlushRx();
                    OpenMV_SetMode(OMV_MODE_APRILTAG);
                    OpenMV_SetTargetTag(1);
                    smooth_Settarget(0, 180, 1000);   /* 腰座转车尾 */
                    dock_tick = g_sys_tick;
                }
                OpenMV_PollLine();
                if (!smoothservo_IsBusy(0)) {
                    /* 腰座到位: 看到 ID=1 立即进倒车(不加帧确认, 响应快) */
                    if (OpenMV_GetData(&omv_d)
                        && omv_d.tag_id == 1
                        && omv_d.distance_cm > 0) {
                        printf("DOCK: found tag1 cx=%d dist=%dcm\r\n",
                               (int)omv_d.cx, (int)omv_d.distance_cm);
                        dock_last_cx = omv_d.cx;
                        dock_last_dist = omv_d.distance_cm;
                        VisualServo_Reset(&visual_servo);   /* 清积分/微分残留 */
                        /* 立即起步: 用本帧数据先算一次倒车速度, 不等下一帧(消除起步空窗) */
                        if (omv_d.distance_cm > 40) {
                            motor_enable = 1;
                            work_mode = MODE_MANUAL;
                            VisualServo_TaskReverse(&visual_servo, &omv_d, target_speed);
                            target_speed[3] += (int16_t)((-target_speed[3]) * DOCK_RR_PCT / 100);
                            /* 右前轮倒车补偿: 低速出力不足, 按比例加大(仅负速度) */
                            if (target_speed[1] < 0)
                                target_speed[1] += (int16_t)(target_speed[1] * DOCK_RF_PCT / 100);
#if DOCK_INNER_OPENLOOP
                            /* 内环开环: 视觉伺服输出直接映射 PWM (同右拐), 跳过编码器速度闭环 */
                            motor_enable = 0;
                            Motor_SetSpeed(0, (int16_t)(motor_dir[0] * target_speed[0]));
                            Motor_SetSpeed(1, (int16_t)(motor_dir[1] * target_speed[1]));
                            Motor_SetSpeed(2, (int16_t)(motor_dir[2] * target_speed[2]));
                            Motor_SetSpeed(3, (int16_t)(motor_dir[3] * target_speed[3]));
#endif
                        }
                        dock_state = 3;
                        dock_tick = g_sys_tick;
                        dock_lost = 0;
                    } else {
                        if (g_sys_tick - dock_tick > 300) {
                            /* 3s没看到: 腰座小范围慢慢扫(±10°);
                               每次扫摆都重发模式命令 — OpenMV 心跳活着≠在 APRILTAG 模式,
                               若模式命令丢失/固件未重启, 只靠掉线检测永远等不到 $TAG */
                            static uint8_t sweep = 0;
                            sweep = !sweep;
                            printf("DOCK: resend MODE (sweep %d)\r\n", (int)sweep);
                            OpenMV_SetMode(OMV_MODE_APRILTAG);
                            OpenMV_SetTargetTag(1);
                            smooth_Settarget(0, sweep ? 170 : 190, 600);
                            dock_tick = g_sys_tick;
                        }
                    }
                }
                /* 总超时: 30s 找不到 tag1 -> 放弃对接恢复巡线 (防无人值守/无串口时卡死) */
                if (g_sys_tick - dock_st2_start > 3000) {
                    printf("DOCK: tag1 not found (30s), skip dock, resume follow\r\n");
                    dock_st2_init = 0;
                    OpenMV_FlushRx();
                    OpenMV_SendCmd("MODE,QRCODE");
                    dock_state = 0;
                    trailer_dock_flag = 0;
                    LineFollow_Reset(&line_follow);
                    work_mode = MODE_LINE_FOLLOW;
                    motor_enable = 1;
                    target_speed[0] = 180; target_speed[1] = 180;
                    target_speed[2] = 180; target_speed[3] = 170;
                    flow_stage = 1;      /* LED 回到巡线 */
                }
            } else if (dock_state == 3) {
                /* 倒车对接: VisualServo_TaskReverse 双PID (横向修正+纵向距离) */
                OpenMV_PollLine();
                if (smoothservo_IsBusy(0)) {
                    /* 腰座还在转向车尾(180°): 等待到位再开始倒车计时,
                       dock_tune/dock_test 直进 st=3 时腰座可能还在转 */
                    dock_tick = g_sys_tick;   /* 刷新计时, 腰座转动期不计丢码 */
                    dock_lost = 0;
                } else if (OpenMV_GetData(&omv_d)
                    && omv_d.tag_id == 1
                    && omv_d.distance_cm > 0) {
                    /* 距离跳变过滤: 30fps 真实变化每帧≤3cm, 单帧跳>8cm = OpenMV 锁到别的
                       id=1 码(多码场景)或坏读数, 整帧丢弃: 不更新 last, 不转向, 维持原速 */
                    if (dock_last_dist > 0
                        && (omv_d.distance_cm > dock_last_dist + 8
                            || omv_d.distance_cm < dock_last_dist - 8)) {
                        /* 丢弃本帧 */
                    } else {
                    dock_tick = g_sys_tick;   /* 记录最后有效码时刻 */
                    dock_lost = 0;
                    dock_last_cx = omv_d.cx;      /* trace 显示真实码位置 */
                    dock_last_dist = omv_d.distance_cm;
                    if (omv_d.distance_cm <= 40) {
                        printf("DOCK: OK reached %dcm\r\n", (int)omv_d.distance_cm);
                        motor_enable = 0;
                        work_mode = MODE_MANUAL;
                        for (i = 0; i < 4; i++) {
                            target_speed[i] = 0;
                            Motor_SetSpeed(i, 0);
                        }
                        if (dock_tune_on) {
                            /* 调参模式: 不挂钩, 前进离开重新对接 */
                            dock_state = 6;
                            dock_tick = g_sys_tick;
                        } else {
                            dock_state = 4;
                            dock_tick = g_sys_tick;
                        }
                    } else {
                        /* 看到码就视觉伺服倒车: 双 PID 同时修正横向(cx→轮子差速)与纵向(距离→倒车速度),
                           腰座在 dist>45cm 时按 cx 偏差比例持续粗对准(不停车),
                           45cm 内交给轮子差速, 防腰座棘轮顶限位 */
                        motor_enable = 1;
                        work_mode = MODE_MANUAL;
                        VisualServo_TaskReverse(&visual_servo, &omv_d, target_speed);
                        /* 比例式开环补偿: 右后轮按速度的 7% 减速, 全速度段补偿量一致 */
                        target_speed[3] += (int16_t)((-target_speed[3]) * DOCK_RR_PCT / 100);
                        /* 右前轮倒车补偿: 低速出力不足过不了死区, 按比例加大(仅负速度) */
                        if (target_speed[1] < 0)
                            target_speed[1] += (int16_t)(target_speed[1] * DOCK_RF_PCT / 100);
#if DOCK_INNER_OPENLOOP
                        /* 内环开环: 视觉伺服输出直接映射 PWM (同右拐), 跳过编码器速度闭环 */
                        motor_enable = 0;
                        Motor_SetSpeed(0, (int16_t)(motor_dir[0] * target_speed[0]));
                        Motor_SetSpeed(1, (int16_t)(motor_dir[1] * target_speed[1]));
                        Motor_SetSpeed(2, (int16_t)(motor_dir[2] * target_speed[2]));
                        Motor_SetSpeed(3, (int16_t)(motor_dir[3] * target_speed[3]));
#endif
                        /* 诊断: 倒车时打印 目标vs实际速度 (定位"为什么前进": 实际为正=轮子在前进!) */
                        {
                            static uint32_t rdbg_tick = 0;
                            int di2;
                            if (g_sys_tick - rdbg_tick >= 20) {
                                rdbg_tick = g_sys_tick;
                                printf("RDK: dist=%d cx=%d tgt=%d,%d,%d,%d real=%d,%d,%d,%d\r\n",
                                       (int)omv_d.distance_cm, (int)omv_d.cx,
                                       (int)target_speed[0], (int)target_speed[1],
                                       (int)target_speed[2], (int)target_speed[3],
                                       (int)(Encoder_getspeed(0) * encoder_dir[0]),
                                       (int)(Encoder_getspeed(1) * encoder_dir[1]),
                                       (int)(Encoder_getspeed(2) * encoder_dir[2]),
                                       (int)(Encoder_getspeed(3) * encoder_dir[3]));
                            }
                        }
                        if ((omv_d.cx < 150 || omv_d.cx > 200)
                            && omv_d.distance_cm > 45   /* 45cm内轮子差速接管, 腰座停步防棘轮顶限位 */
                            && !smoothservo_IsBusy(0)) {
                            int16_t waist_now = (int16_t)smooth_GetCurrentAngle(0);
                            int16_t waist_target;
                            waist_target = waist_now
                                + (int16_t)(DOCK_WAIST_ALIGN_DIR)
                                  * ((omv_d.cx > 200) ? -5 : 5);
                            if (waist_target < 140) waist_target = 140;
                            if (waist_target > 220) waist_target = 220;
                            smooth_Settarget(0, (uint16_t)waist_target, 300);
                        }
                    }
                    }   /* end 跳变过滤-有效帧 */
                } else {
                    /* 无新数据(OpenMV 30fps 帧间隙, 主循环飞快):
                       保持最后速度继续退, 绝不清零(否则"输出→清零"反复, 车不动);
                       丢码但已到目标附近(最后距离≤40) → 直接判到达, 不进找码 */
                    {
                        uint32_t miss = g_sys_tick - dock_tick;
                        dock_lost = (miss > 255) ? 255 : (uint8_t)miss;
                        if (dock_last_dist > 0 && dock_last_dist <= 40) {
                            /* 已退到位(≤40cm): 直接判到达 */
                            printf("DOCK: OK reached (lost, dist=%d)\r\n",
                                   (int)dock_last_dist);
                            motor_enable = 0;
                            work_mode = MODE_MANUAL;
                            for (i = 0; i < 4; i++) {
                                target_speed[i] = 0;
                                Motor_SetSpeed(i, 0);
                            }
                            if (dock_tune_on) {
                                dock_state = 6;
                                dock_tick = g_sys_tick;
                            } else {
                                dock_state = 4;
                                dock_tick = g_sys_tick;
                            }
                        } else if (miss > 100) {
                            /* 丢码: 刹车停车 (2026-08-24 二次修复: 之前 miss>30=0.3s 太紧,
                               OpenMV APRILTAG 检测 ~200ms/帧 + 偶发延迟 → 正常帧间隙被误判丢码,
                               车刚起步倒车就被刹 → "不往后走". 放宽到 100 ticks=1s, 覆盖 5 帧+延迟,
                               真丢码 1s 后才刹; 滑行 1s 远小于原 2.5s) */
                            printf("DOCK: lost, brake (last=%d)\r\n",
                                   (int)dock_last_dist);
                            motor_enable = 0;
                            work_mode = MODE_MANUAL;
                            for (i = 0; i < 4; i++) {
                                target_speed[i] = 0;
                                Motor_SetSpeed(i, 0);
                            }
                            dock_st2_init = 0;   /* 回找码: 重置30s计时与初始化 */
                            dock_state = 2;
                            dock_lost = 0;
                        }
                        /* miss <= 250: 保持 target_speed 不变, 什么都不做 */
                    }
                }
            } else if (dock_state == 4) {
                /* 挂钩: 先确保轮子停(防 dock_state3 直驱倒车残留), 再挂 */
                motor_enable = 0;
                work_mode = MODE_MANUAL;
                for (i = 0; i < 4; i++) {
                    target_speed[i] = 0;
                    Motor_SetSpeed(i, 0);
                }
                printf("DOCK: hooking trailer\r\n");
                Action_HookTrailer();
                Delay_ms(500);
                trailer_docked = 1;
                dock_state = 8;   /* 腰座复位(车停着) → 再回轨 */
                printf("DOCK: OK, trailer hooked (trailer_docked=1)\r\n");
            } else if (dock_state == 8) {
                /* 腰座复位: 车保持停住, 腰座先转回 home 完成后再回轨 */
                if (!dock_done_once) {
                    dock_done_once = 1;
                    flow_stage = 3;         /* LED 中闪 = 工位作业 */
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        PID_Reset(&pid_motor[i]);
                        Motor_SetSpeed(i, 0);
                    }
                    OpenMV_FlushRx();
                    OpenMV_SendCmd("MODE,QRCODE");
                    smooth_Settarget(0, ARM_HOME_WAIST, 1500);   /* 腰座转回 home(5°) */
                    dock_st7_init = 0;
                    dock_st7_phase = 0;
                    printf("DOCK: hooked, waist reset to home\r\n");
                } else if (!smoothservo_IsBusy(0)) {
                    /* 腰座复位完成 → 回轨 */
                    dock_state = 7;
                    dock_tick = g_sys_tick;
                    printf("DOCK: waist done, return to line\r\n");
                }
            } else if (dock_state == 5) {
                /* 巡线完成态: AI 流水线 gate (dock_state==5 激活). 不做动作 */
            } else if (dock_state == 7) {
                /* 回轨: 腰座已复位(dock_state8), 直接巡线 — 不用掉头180°(用户要求).
                   右前/左后开环(编码器real乱→PID乱→乱转, lfdbg证实) */
                dock_st7_init = 0;
                trailer_dock_flag = 0;
                encoder_max[1] = 0;   /* 右前开环 */
                encoder_max[2] = 0;   /* 左后开环 */
                for (i = 0; i < 4; i++) {
                    target_speed[i] = 180;   /* 初始前进速度, 巡线PID接管 */
                    PID_Reset(&pid_motor[i]);
                }
                LineFollow_Reset(&line_follow);
                work_mode = MODE_LINE_FOLLOW;
                motor_enable = 1;
                dock_state = 5;
                printf("DOCK: line-follow resumed\r\n");
            } else if (dock_state == 6) {
                /* 调参循环: 前进远离从车(摄像头朝车尾仍可见码), 距离>45cm 再倒车
                   速度不要太猛(200太快会把码甩丢), 用 120 温和前进 */
                OpenMV_PollLine();
                motor_enable = 1;
                work_mode = MODE_MANUAL;
                target_speed[0] = 120; target_speed[1] = 120;
                target_speed[2] = 120; target_speed[3] = 120;
                /* 前进同样加右后轮比例补偿(减速7%): 开环右后轮跑快会把车带偏, 码甩出视野 */
                target_speed[3] += (int16_t)((-target_speed[3]) * DOCK_RR_PCT / 100);
                if (OpenMV_GetData(&omv_d)
                    && omv_d.tag_id == 1
                    && omv_d.distance_cm > 0) {
                    dock_last_cx = omv_d.cx;
                    dock_last_dist = omv_d.distance_cm;
                    dock_tick = g_sys_tick;   /* 刷新"最近看到码"时刻, 防误判离开丢码 */
                    if (omv_d.distance_cm > 45) {
                        if (++dock_offline_cnt >= 3) {   /* 3帧消抖 */
                            printf("DOCK: tune away %dcm, reverse again\r\n",
                                   (int)omv_d.distance_cm);
                            for (i = 0; i < 4; i++) {
                                target_speed[i] = 0;
                                Motor_SetSpeed(i, 0);
                            }
                            dock_offline_cnt = 0;
                            dock_state = 3;
                            dock_tick = g_sys_tick;
                            dock_lost = 0;
                        }
                    } else {
                        dock_offline_cnt = 0;
                    }
                } else if (g_sys_tick - dock_tick > 500)
                    /* 5s没看到码: 刹车回找码阶段 (500 tick @ 100Hz = 5s) */
                    printf("DOCK: tune lost while away, brake\r\n");
                    motor_enable = 0;
                    work_mode = MODE_MANUAL;
                    for (i = 0; i < 4; i++) {
                        target_speed[i] = 0;
                        Motor_SetSpeed(i, 0);
                    }
                    dock_state = 2;
                    dock_tick = g_sys_tick;
                    dock_lost = 0;
                }
            }
        }
#endif /* ==== END dock state machine disabled ==== */

        TaskQueue_Process(&task_queue);

        /* trace: 周期调试输出 (dock_tune 定位"不动"用) */
        if (trace_enable && dock_state != 0) {
            static uint32_t trace_tick = 0;
            if (g_sys_tick - trace_tick >= 100) {   /* 100ms: 原来10ms一行≈7.7ms printf,
                                                        UART5被占死 → OpenMV帧被ORE冲掉,
                                                        刚found下一帧就丢 → 误判lost */
                trace_tick = g_sys_tick;
                printf("TRACE: st=%d lost=%d en=%d wm=%d tune=%d cx=%d dist=%d waist=%d spd=%d,%d,%d,%d\r\n",
                       (int)dock_state, (int)dock_lost,
                       (int)motor_enable, (int)work_mode, (int)dock_tune_on,
                       (int)dock_last_cx, (int)dock_last_dist,
                       (int)smooth_GetCurrentAngle(0),
                       (int)target_speed[0], (int)target_speed[1],
                       (int)target_speed[2], (int)target_speed[3]);
            }
        }

        }
    }
	}
