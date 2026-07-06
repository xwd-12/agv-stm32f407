#include "stm32f4xx.h"
#include "smooth_servo.h"
#include "delay.h"
#include "arm_config.h"
#include "action_group.h"
#include "uart.h"
#include <stdio.h>

/* ===== 运行时参数(默认值 = 原 #define 值) ===== */
uint16_t act_angle[ACT_ANGLE_COUNT] = {
    60,    /* ACT_APPROACH_WAIST     */
    110,   /* ACT_APPROACH_SHOULDER  */
    10,    /* ACT_APPROACH_ELBOW     */
     0,    /* ACT_GRIPPER_CLOSE      */
    88,    /* ACT_LIFT_SHOULDER      */
    110,   /* ACT_PLACE_WAIST        */
    80,   /* ACT_PLACE_SHOULDER     */
    140,   /* ACT_PLACE_ELBOW        */
    180,   /* ACT_GRIPPER_OPEN       */
    100,   /* ACT_RESET_SHOULDER     */
    95,    /* ACT_RESET_ELBOW        */
    160    /* ACT_PLACE_OPPOSITE     */
};

uint16_t act_time[ACT_TIME_COUNT] = {
    3000,  /* ACT_TIME_APPROACH    */
    1500,  /* ACT_TIME_GRIP        */
    2500,  /* ACT_TIME_LIFT        */
    3000,  /* ACT_TIME_PLACE       */
    2000,  /* ACT_TIME_RETRACT     */
    2000   /* ACT_TIME_PLACE_WAIST */
};

static const char *act_angle_names[] = {
    "approach_waist",
    "approach_shoulder",
    "approach_elbow",
    "gripper_close",
    "lift_shoulder",
    "place_waist",
    "place_shoulder",
    "place_elbow",
    "gripper_open",
    "reset_shoulder",
    "reset_elbow",
    "place_opposite"
};

static const char *act_time_names[] = {
    "time_approach",
    "time_grip",
    "time_lift",
    "time_place",
    "time_retract",
    "time_place_waist"
};

#define SERVO_WAIST    0
#define SERVO_SHOULDER 1
#define SERVO_ELBOW    2
#define SERVO_GRIPPER  3
#define SERVO_HOOK     4   /* 挂钩舵机 PA10 */

/* Bug#7: 紧急停止标志 (UART收到stop指令时置位) */
static uint8_t g_action_estop = 0;

/**
 * Bug#7: 检查UART环形缓冲区是否有"stop"紧急停止指令
 * 在阻塞等待舵机期间周期性调用, 防止长时间无响应
 * 返回 1=检测到stop指令, 0=无
 */
static int check_uart_estop(void)
{
    /* 只扫描环形缓冲区找 "stop", 不消费任何字节
     * (避免抢走主循环正在拼的命令行) */
    if (uart_ring_peek("stop")) {
        g_action_estop = 1;
        smoothservo_EmergencyStop();
        printf("ESTOP: stop via UART during action\r\n");
        return 1;
    }
    return 0;
}

void Action_ParamInit(void)
{
    /* 使用文件作用域的默认值, 未来可扩展为从Flash加载 */
}

void Action_SetAngle(uint8_t id, uint16_t value)
{
    if (id < ACT_ANGLE_COUNT) {
        act_angle[id] = value;
        printf("act_angle[%d](%s) = %d\r\n", id, act_angle_names[id], value);
    }
}

void Action_SetTime(uint8_t id, uint16_t value)
{
    if (id < ACT_TIME_COUNT) {
        act_time[id] = value;
        printf("act_time[%d](%s) = %d ms\r\n", id, act_time_names[id], value);
    }
}

void Action_ShowParams(void)
{
    uint8_t i;
    printf("=== Action Angles ===\r\n");
    for (i = 0; i < ACT_ANGLE_COUNT; i++)
        printf("  [%d] %-20s = %d\r\n", i, act_angle_names[i], act_angle[i]);
    printf("=== Action Times ===\r\n");
    for (i = 0; i < ACT_TIME_COUNT; i++)
        printf("  [%d] %-20s = %d ms\r\n", i, act_time_names[i], act_time[i]);
}

/* 等待单个舵机完成 (Bug#7: 轮询UART紧急停止) */
static void WaitForServo(uint8_t id)
{
    if (g_action_estop) return;
    while (smoothservo_IsBusy(id)) {
        Delay_ms(10);
        if (check_uart_estop()) return;
        Delay_ms(10);
    }
}

/* 等待所有舵机完成 (Bug#7: 轮询UART紧急停止) */
static void WaitforAllservo(uint16_t delay_ms)
{
    uint16_t chunk;
    if (g_action_estop) return;
    if (delay_ms > 50) delay_ms = 50;  /* 最多等50ms就检查一次UART */
    while (smoothservo_IsBusy(SERVO_WAIST) ||
           smoothservo_IsBusy(SERVO_SHOULDER) ||
           smoothservo_IsBusy(SERVO_ELBOW) ||
           smoothservo_IsBusy(SERVO_GRIPPER)) {
        /* 分段延时, 每小段后检查UART */
        for (chunk = 0; chunk < delay_ms; chunk += 10) {
            Delay_ms(10);
            if (check_uart_estop()) return;
        }
    }
}

/* 复位所有关节 */
void Action_Reset(void)
{
    smooth_Settarget(SERVO_WAIST,    ARM_HOME_WAIST,          2500);
    Delay_ms(200);
    smooth_Settarget(SERVO_SHOULDER, ARM_HOME_SHOULDER,      2500);
    Delay_ms(200);
    smooth_Settarget(SERVO_ELBOW,    ARM_HOME_ELBOW,         2500);
    Delay_ms(200);
    smooth_Settarget(SERVO_GRIPPER,  ARM_HOME_GRIPPER, 500);
    WaitforAllservo(300);
}

uint8_t Action_ISdle(void)
{
    return !(smoothservo_IsBusy(SERVO_WAIST) ||
             smoothservo_IsBusy(SERVO_SHOULDER) ||
             smoothservo_IsBusy(SERVO_ELBOW) ||
             smoothservo_IsBusy(SERVO_GRIPPER));
}

void Action_Init(void)
{
    Action_Reset();
    while (!Action_ISdle());
}

/* 抓取动作 (使用运行时参数) */
void Action_Grasp(void)
{
    g_action_estop = 0;  /* Bug#7: 清除上次残留 */

    /* 1. 腰座旋转到目标方向 */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_APPROACH_WAIST], act_time[ACT_TIME_APPROACH]);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */

    /* 2. 大臂小臂下探，同时空中张开夹爪 */
    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_APPROACH_SHOULDER], act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_APPROACH_ELBOW], 1200);
    /* 手臂下探的同时张开夹爪，不等手臂到位 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_GRIPPER, 0, 200);
    Delay_ms(250);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_OPEN], 600);
    /* 等小臂到位即合爪，大臂继续下探(提前约1s) */
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(900);
    if (g_action_estop) return;  /* Bug#7 */
    /* 3. 闭合夹爪抓取 */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 800);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;  /* Bug#7 */
    /* 等大臂也到位 */
    WaitforAllservo(200);
    if (g_action_estop) return;  /* Bug#7 */

    /* 6. 抬起(用复位角度) */
    smooth_Settarget(SERVO_ELBOW,    ARM_HOME_ELBOW,    2000);
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_SHOULDER, ARM_HOME_SHOULDER, 2000);
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */
    /* 腰座转回复位 */
    smooth_Settarget(SERVO_WAIST, ARM_HOME_WAIST, 2000);
    WaitForServo(SERVO_WAIST);

}

/* 放置动作: 下探→松爪→抬起→复位时闭合夹爪 */
void Action_Place(uint16_t waist_angle)
{
    (void)waist_angle;  /* 使用和抓取相同的角度, 忽略参数 */
    g_action_estop = 0;  /* Bug#7: 清除上次残留 */

    /* 1. 腰座旋转到目标方向(同抓取) */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_APPROACH_WAIST], act_time[ACT_TIME_APPROACH]);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */

    /* 2. 大臂小臂下探(同抓取) */
    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_APPROACH_SHOULDER], act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_APPROACH_ELBOW], 1200);
    WaitforAllservo(200);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(300);
    if (g_action_estop) return;  /* Bug#7 */

    /* 3. 放开夹爪, 保持张开 */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_OPEN], 800);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(300);
    if (g_action_estop) return;  /* Bug#7 */

    /* 4. 抬起+复位(保持夹爪张开) */
    smooth_Settarget(SERVO_ELBOW,    ARM_HOME_ELBOW,    2000);
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_SHOULDER, ARM_HOME_SHOULDER, 2000);
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_WAIST, ARM_HOME_WAIST, 2000);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */

    /* 5. 复位完成后闭合夹爪 */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 800);
    WaitForServo(SERVO_GRIPPER);
}

/* 完整抓取→放置流程 */
void Action_GraspAndPlace(void)
{
    /* ===== 1. 向右转，抓取 ===== */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_APPROACH_WAIST], act_time[ACT_TIME_PLACE_WAIST]);
    WaitForServo(SERVO_WAIST);
    Delay_ms(500);

    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_APPROACH_SHOULDER], act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_APPROACH_ELBOW], 1200);
    WaitforAllservo(200);
    Delay_ms(800);

    smooth_Settarget(SERVO_ELBOW,    act_angle[ACT_RESET_ELBOW],    act_time[ACT_TIME_LIFT]);
    Delay_ms(500);
    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_LIFT_SHOULDER], act_time[ACT_TIME_LIFT]);
    WaitForServo(SERVO_ELBOW);
    WaitForServo(SERVO_SHOULDER);
    Delay_ms(500);

    /* ===== 2. 转到相反方向，投放 ===== */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_PLACE_OPPOSITE], act_time[ACT_TIME_PLACE_WAIST]);
    WaitForServo(SERVO_WAIST);
    Delay_ms(200);

    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_PLACE_SHOULDER], act_time[ACT_TIME_PLACE]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_PLACE_ELBOW],    act_time[ACT_TIME_PLACE]);
    WaitforAllservo(200);
    Delay_ms(500);

    smooth_Settarget(SERVO_ELBOW,    act_angle[ACT_RESET_ELBOW],    act_time[ACT_TIME_RETRACT]);
    smooth_Settarget(SERVO_SHOULDER, act_angle[ACT_RESET_SHOULDER], act_time[ACT_TIME_RETRACT]);
    WaitforAllservo(400);

    /* ===== 3. 复位 ===== */
    Action_Reset();
}

/* 挂接从车: 挂钩舵机从90°(脱钩)转到0°(锁定) */
void Action_HookTrailer(void)
{
    g_action_estop = 0;
    smooth_Settarget(SERVO_HOOK, ARM_HOOK_LOCK, 500);
    WaitForServo(SERVO_HOOK);
    if (g_action_estop) return;
    printf("Trailer hooked\r\n");
}

/* 脱开从车: 挂钩舵机从0°(锁定)转回90°(脱钩) */
void Action_UnhookTrailer(void)
{
    g_action_estop = 0;
    smooth_Settarget(SERVO_HOOK, ARM_HOME_HOOK, 500);
    WaitForServo(SERVO_HOOK);
    if (g_action_estop) return;
    printf("Trailer unhooked\r\n");
}
