#include "stm32f4xx.h"
#include "servo.h"
#include "pwm.h"
#include "stdbool.h"
#include "arm_config.h"

#define SMOOTH_SERVO_NUM  5
#define MAX_ANGULAR_VEL   180.0f   // 最大角速度(度/秒), 五次多项式峰值约为平均值1.875倍

// 间歇保持: 到位后关断避免死区振荡, 周期性短暂上电修正下垂
#define HOLD_TICKS   50   // 到位后保持 500ms
#define OFF_TICKS    500  // 关断 5s
#define ON_TICKS      50  // 上电 500ms 修正下垂

// 状态枚举
typedef enum {
    SS_IDLE = 0,
    SS_MOVING,
    SS_ESTOP
} SmoothState;

static float      current_angle[SMOOTH_SERVO_NUM];
static float      target_angle[SMOOTH_SERVO_NUM];
static float      start_angle[SMOOTH_SERVO_NUM];
static int32_t    steps_total[SMOOTH_SERVO_NUM];
static int32_t    steps_elapsed[SMOOTH_SERVO_NUM];
static SmoothState state[SMOOTH_SERVO_NUM];
static int16_t    idle_timer[SMOOTH_SERVO_NUM];  // 间歇计时器
static uint8_t    idle_phase[SMOOTH_SERVO_NUM];  // 0=保持(ON), 1=关断(OFF)

void smooth_Init(void)
{
    int i;
    for (i = 0; i < SMOOTH_SERVO_NUM; i++) {
        current_angle[i] = 90.0f;
        target_angle[i]  = 90.0f;
        start_angle[i]   = 90.0f;
    }
    // 同步 Servo_Init 的初始位置
    current_angle[0] = (float)ARM_HOME_WAIST;    target_angle[0] = (float)ARM_HOME_WAIST;
    current_angle[1] = (float)ARM_HOME_SHOULDER; target_angle[1] = (float)ARM_HOME_SHOULDER;
    current_angle[2] = (float)ARM_HOME_ELBOW;    target_angle[2] = (float)ARM_HOME_ELBOW;
    current_angle[3] = (float)ARM_HOME_GRIPPER;  target_angle[3] = (float)ARM_HOME_GRIPPER;
    current_angle[4] = (float)ARM_HOME_HOOK;     target_angle[4] = (float)ARM_HOME_HOOK;
    for (i = 0; i < SMOOTH_SERVO_NUM; i++) {
        steps_total[i]   = 0;
        steps_elapsed[i] = 0;
        state[i]         = SS_IDLE;
        idle_timer[i]    = HOLD_TICKS;
        idle_phase[i]    = 0;
        // 物理位置由 Servo_Init + Action_Reset 接管, 这里不写 PWM
    }
}

int16_t smooth_Settarget(uint8_t id, uint16_t target_deg, uint16_t time_ms)
{
    float total_distance;
    int32_t total_steps;
    int32_t min_steps;
    float peak_vel;
		float dist_abs;

    if (id >= SMOOTH_SERVO_NUM)
        return 0;

    // 急停状态拒绝新指令
    if (state[id] == SS_ESTOP)
        return -1;

    target_angle[id] = (float)target_deg;
    total_distance   = target_angle[id] - current_angle[id];

    // 已在目标位置(误差<0.5度): 跳过运动，直接回IDLE
    {
        float d = total_distance;
        if (d < 0.0f) d = -d;
        if (d < 0.5f) {
            Servo_PWMEnable(id, 1);
            current_angle[id] = target_angle[id];
            state[id] = SS_IDLE;
            idle_phase[id] = 0;
            idle_timer[id] = HOLD_TICKS;
            return 1;
        }
    }

    // 速度限制: 如果指令速度超过 MAX_ANGULAR_VEL, 自动延长 time_ms
    if (time_ms < 10) time_ms = 10;
     dist_abs = total_distance;
    if (dist_abs < 0.0f) dist_abs = -dist_abs;
    peak_vel = 1.875f * dist_abs / ((float)time_ms / 1000.0f);
    if (peak_vel > MAX_ANGULAR_VEL) {
        float min_time = 1.875f * dist_abs / MAX_ANGULAR_VEL * 1000.0f;
        if (min_time > 65535.0f) min_time = 65535.0f;
        time_ms = (uint16_t)min_time;
        if (time_ms < 10) time_ms = 10;
    }

    total_steps = (int32_t)(time_ms / 10);
    if (total_steps < 1) total_steps = 1;

    Servo_PWMEnable(id, 1);          // 运动前重开PWM
    idle_phase[id]    = 0;
    start_angle[id]   = current_angle[id];
    steps_total[id]   = total_steps;
    steps_elapsed[id] = 0;
    state[id]         = SS_MOVING;

    return 1;
}

bool smoothservo_IsBusy(uint8_t id)
{
    if (id >= SMOOTH_SERVO_NUM)
        return false;
    return (state[id] == SS_MOVING);
}

bool smoothservo_AnyBusy(void)
{
    int i;
    for (i = 0; i < SMOOTH_SERVO_NUM; i++) {
        if (state[i] == SS_MOVING)
            return true;
    }
    return false;
}

uint8_t smoothservo_GetState(uint8_t id)
{
    if (id >= SMOOTH_SERVO_NUM)
        return 0xFF;
    return (uint8_t)state[id];
}

void smoothservo_EmergencyStop(void)
{
    int i;
    for (i = 0; i < SMOOTH_SERVO_NUM; i++) {
        if (state[i] == SS_MOVING) {
            state[i]         = SS_ESTOP;
            steps_elapsed[i] = 0;
            steps_total[i]   = 0;
        }
    }
}

/* 停止单个通道的平滑运动，回到IDLE状态 */
void smoothservo_StopOne(uint8_t id)
{
    if (id >= SMOOTH_SERVO_NUM)
        return;
    if (state[id] == SS_MOVING) {
        state[id]         = SS_IDLE;
        steps_elapsed[id] = 0;
        steps_total[id]   = 0;
        /* 保持当前位置，不清除idle_timer */
    }
}

void smoothservo_TimerHandle(void)
{
    int i;
    for (i = 0; i < SMOOTH_SERVO_NUM; i++) {
        if (state[i] == SS_MOVING) {
            // 运动中: 插值更新
        } else if (state[i] == SS_IDLE) {
            // 大臂(ID=1)+小臂(ID=2)一直供电
            if (i == 1 || i == 2) {
                continue;
            }
            if (i == 3 && idle_phase[i] == 1) {
                continue;
            }
            // 腰座(ID=0)+挂钩(ID=4): 到位后断电永久保持
            if ((i == 0 || i == 4) && idle_phase[i] == 1) {
                continue;
            }
            if (--idle_timer[i] <= 0) {
                if (idle_phase[i] == 0) {
                    Servo_PWMEnable(i, 0);
                    if (i == 0 || i == 4) {
                        idle_timer[i] = 0x7FFF;   // 腰座永久断电
                    } else {
                        idle_phase[i] = 1;
                        idle_timer[i] = OFF_TICKS;
                    }
                } else {
                    Servo_SetAngle(i, (uint16_t)current_angle[i]);
                    Servo_PWMEnable(i, 1);
                    idle_phase[i] = 0;
                    idle_timer[i] = ON_TICKS;
                }
            }
            continue;
        } else {
            continue;
        }

        steps_elapsed[i]++;

        if (steps_elapsed[i] >= steps_total[i]) {
            // 运动完成
            current_angle[i] = target_angle[i];
            Servo_SetAngle(i, (uint16_t)current_angle[i]);
            state[i]         = SS_IDLE;
            idle_phase[i]    = 0;
            idle_timer[i]    = HOLD_TICKS;  // 先保持500ms再关断
        } else {
            // 五次多项式 smoothstep: 速度/加速度在起止点均为0
            float t = (float)steps_elapsed[i] / (float)steps_total[i];
            float t3 = t * t * t;
            float eased = t3 * (t * (t * 6.0f - 15.0f) + 10.0f);
            current_angle[i] = start_angle[i] + eased * (target_angle[i] - start_angle[i]);
            Servo_SetAngle(i, (uint16_t)current_angle[i]);
        }
    }
}

float smooth_GetCurrentAngle(uint8_t id)
{
    if (id >= SMOOTH_SERVO_NUM)
        return 0.0f;
    return current_angle[id];
}
