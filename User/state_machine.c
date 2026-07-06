#include "stm32f4xx.h"
#include "state_machine.h"
#include "action_group.h"
#include "smooth_servo.h"
#include "enconder.h"
#include "line_follow.h"
 #include "visual_servo.h"
#include "commend_openmv.h"

// 超时阈值(10ms tick 计数)
#define TIMEOUT_GRASP   3000  // 抓取超时 30s
#define TIMEOUT_PLACE   3000  // 投放超时 30s
#define TIMEOUT_RESET   3000  // 复位超时 30s
#define DEBOUNCE_TICKS  5     // 状态切换防抖 50ms
#define TASK_TIMEOUT_TICKS 3000  // TaskNav 巡线超时 30s

static ArmState arm_state        = ARM_IDLE;
static ArmState arm_state_next   = ARM_IDLE;
static uint16_t tick_counter     = 0;
static uint16_t timeout_limit    = 0;
static uint8_t  state_stable_cnt = 0;

void ArmSM_Init(void)
{
    arm_state        = ARM_IDLE;
    arm_state_next   = ARM_IDLE;
    tick_counter     = 0;
    timeout_limit    = 0;
    state_stable_cnt = 0;
}

ArmState ArmSM_GetState(void)
{
    return arm_state;
}

uint8_t ArmSM_IsBusy(void)
{
    return (arm_state != ARM_IDLE && arm_state != ARM_ESTOP);
}

uint8_t ArmSM_RequestGrasp(void)
{
    if (arm_state != ARM_IDLE)
        return 0;
    arm_state_next   = ARM_GRASPING;
    timeout_limit    = TIMEOUT_GRASP;
    tick_counter     = 0;
    state_stable_cnt = 0;
    return 1;
}

uint8_t ArmSM_RequestPlace(uint16_t waist_angle)
{
    if (arm_state != ARM_IDLE)
        return 0;
    (void)waist_angle;           // angle 由 command 层直接传给 Action_Place
    arm_state_next   = ARM_PLACING;
    timeout_limit    = TIMEOUT_PLACE;
    tick_counter     = 0;
    state_stable_cnt = 0;
    return 1;
}

uint8_t ArmSM_RequestReset(void)
{
    if (arm_state == ARM_ESTOP) {
        // 急停后复位: 先清除急停
        arm_state        = ARM_IDLE;
        arm_state_next   = ARM_IDLE;
        state_stable_cnt = 0;
    }
    if (arm_state != ARM_IDLE)
        return 0;
    arm_state_next   = ARM_RESETTING;
    timeout_limit    = TIMEOUT_RESET;
    tick_counter     = 0;
    state_stable_cnt = 0;
    return 1;
}

void ArmSM_EmergencyStop(void)
{
    smoothservo_EmergencyStop();
    arm_state        = ARM_ESTOP;
    arm_state_next   = ARM_ESTOP;
    state_stable_cnt = 0;
}

// 运动完成后由 command 层调用, 将状态切回 IDLE
void ArmSM_NotifyComplete(void)
{
    if (arm_state == ARM_GRASPING || arm_state == ARM_PLACING || arm_state == ARM_RESETTING) {
        arm_state        = ARM_IDLE;
        arm_state_next   = ARM_IDLE;
        state_stable_cnt = 0;
        tick_counter     = 0;
    }
}

// 每10ms调用一次 (ISR上下文, 不能有阻塞调用)
void ArmSM_Tick10ms(void)
{
    // 防抖: 目标状态保持 DEBOUNCE_TICKS 次不变才切换
    if (arm_state != arm_state_next) {
        state_stable_cnt++;
        if (state_stable_cnt >= DEBOUNCE_TICKS) {
            arm_state        = arm_state_next;
            state_stable_cnt = 0;
            tick_counter     = 0;
        }
        return;
    }

    // 超时检测: 任何非空闲/非急停状态
    tick_counter++;
    if (tick_counter >= timeout_limit && timeout_limit > 0
            && arm_state != ARM_IDLE && arm_state != ARM_ESTOP) {
        smoothservo_EmergencyStop();
        arm_state        = ARM_ESTOP;
        arm_state_next   = ARM_ESTOP;
    }
}

const char* ArmSM_StateName(ArmState s)
{
    switch (s) {
    case ARM_IDLE:      return "IDLE";
    case ARM_GRASPING:  return "GRASP";
    case ARM_PLACING:   return "PLACE";
    case ARM_RESETTING: return "RESET";
    case ARM_ESTOP:     return "ESTOP";
    default:            return "???";
    }
}

// ======================== AGV Task Navigation State Machine ========================

extern volatile uint8_t  work_mode;
extern volatile int16_t  target_speed[4];
extern Line_follow_Handle line_follow;
extern volatile uint32_t g_sys_tick;

static TaskNavState task_state    = TASK_IDLE;
static int32_t      task_dist_a   = 0;     // encoder counts to point A
static int32_t      task_dist_b   = 0;     // encoder counts from A to B
static int32_t      task_enc_start[4] = {0, 0, 0, 0};
static uint32_t     task_check_tick  = 0;
static uint32_t     task_timeout_tick = 0; // 巡线超时计时起点

void TaskNav_Init(void)
{
    task_state        = TASK_IDLE;
    task_dist_a       = 0;
    task_dist_b       = 0;
    task_check_tick   = 0;
    task_timeout_tick = 0;
}

void TaskNav_Start(int32_t dist_a, int32_t dist_b)
{
    if (task_state != TASK_IDLE)
        return;

    if (dist_a <= 0 || dist_b <= 0) {
        printf("Task: invalid distance, must be > 0\r\n");
        return;
    }

    task_dist_a = dist_a;
    task_dist_b = dist_b;

    // snapshot encoder start positions (motors 0,1,2; motor 3 encoder broken)
    task_enc_start[0] = Encoder_GetPosition(0);
    task_enc_start[1] = Encoder_GetPosition(1);
    task_enc_start[2] = Encoder_GetPosition(2);
    task_enc_start[3] = 0;

    task_state        = TASK_GO_TO_A;
    task_check_tick   = g_sys_tick;
    task_timeout_tick = g_sys_tick;
    work_mode         = MODE_LINE_FOLLOW;
    printf("Task: GO_TO_A, target=%ld counts\r\n", (long)dist_a);
}

void TaskNav_Stop(void)
{
    int i;
    task_state = TASK_IDLE;
    work_mode  = MODE_MANUAL;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;
    printf("Task stopped\r\n");
}

TaskNavState TaskNav_GetState(void)
{
    return task_state;
}

int32_t TaskNav_GetProgress(void)
{
    int32_t avg;
    if (task_state == TASK_IDLE || task_state == TASK_DONE)
        return 0;
    avg = (
        (Encoder_GetPosition(0) - task_enc_start[0]) +
        (Encoder_GetPosition(1) - task_enc_start[1]) +
        (Encoder_GetPosition(2) - task_enc_start[2])
    ) / 3;
    return avg;
}

int32_t TaskNav_GetTarget(void)
{
    if (task_state == TASK_GO_TO_A || task_state == TASK_GRASP)
        return task_dist_a;
    if (task_state == TASK_GO_TO_B || task_state == TASK_PLACE)
        return task_dist_b;
    return 0;
}

// TaskNav_Process: call in main loop. Non-blocking except for arm actions.
void TaskNav_Process(void)
{
    uint8_t i;
    int32_t avg_dist;

    switch (task_state)
    {

    case TASK_IDLE:
    case TASK_DONE:
        return;

    case TASK_GO_TO_A:
        // throttle: check every 50ms (5 ticks)
        if (g_sys_tick - task_check_tick < 5)
            return;
        task_check_tick = g_sys_tick;

        // timeout: 30s without reaching target
        if (g_sys_tick - task_timeout_tick > TASK_TIMEOUT_TICKS) {
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            work_mode  = MODE_MANUAL;
            task_state = TASK_IDLE;
            printf("Task timeout at GO_TO_A\r\n");
            return;
        }

        avg_dist = (
            (Encoder_GetPosition(0) - task_enc_start[0]) +
            (Encoder_GetPosition(1) - task_enc_start[1]) +
            (Encoder_GetPosition(2) - task_enc_start[2])
        ) / 3;

        if (avg_dist >= task_dist_a) {
            // arrived at A: stop motors, prepare for grasp
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            work_mode  = MODE_MANUAL;
            task_state = TASK_GRASP;
            printf("Arrived at A (%ld counts)\r\n", (long)avg_dist);
        }
        return;

    case TASK_GRASP:
        // arm actions are blocking (they use Delay_ms internally)
        // safe in main loop, not ISR
        // Bug#3: 检查 ArmSM 返回值, 臂忙时跳过
        if (ArmSM_RequestGrasp()) {
            Action_Grasp();
            ArmSM_NotifyComplete();
        } else {
            printf("TaskNav: arm busy, skip grasp\r\n");
        }

        // snapshot encoder for segment B (relative from A)
        task_enc_start[0] = Encoder_GetPosition(0);
        task_enc_start[1] = Encoder_GetPosition(1);
        task_enc_start[2] = Encoder_GetPosition(2);

        task_state        = TASK_GO_TO_B;
        task_check_tick   = g_sys_tick;
        task_timeout_tick = g_sys_tick;
        work_mode         = MODE_LINE_FOLLOW;
        printf("Grasp done, GO_TO_B target=%ld counts\r\n", (long)task_dist_b);
        return;

    case TASK_GO_TO_B:
        if (g_sys_tick - task_check_tick < 5)
            return;
        task_check_tick = g_sys_tick;

        // timeout: 30s without reaching target
        if (g_sys_tick - task_timeout_tick > TASK_TIMEOUT_TICKS) {
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            work_mode  = MODE_MANUAL;
            task_state = TASK_IDLE;
            printf("Task timeout at GO_TO_B\r\n");
            return;
        }

        avg_dist = (
            (Encoder_GetPosition(0) - task_enc_start[0]) +
            (Encoder_GetPosition(1) - task_enc_start[1]) +
            (Encoder_GetPosition(2) - task_enc_start[2])
        ) / 3;

        if (avg_dist >= task_dist_b) {
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            work_mode  = MODE_MANUAL;
            task_state = TASK_PLACE;
            printf("Arrived at B (%ld counts)\r\n", (long)avg_dist);
        }
        return;

    case TASK_PLACE:
        /* Bug#3: 检查 ArmSM 返回值, 臂忙时跳过 */
        if (ArmSM_RequestPlace(110)) {
            Action_Place(110);
            ArmSM_NotifyComplete();
        } else {
            printf("TaskNav: arm busy, skip place\r\n");
        }
        task_state = TASK_DONE;
        work_mode = MODE_LINE_FOLLOW;
        printf("Place done, resume line following\r\n");
        return;
    }
}

const char* TaskNav_StateName(TaskNavState s)
{
    switch (s) {
    case TASK_IDLE:     return "IDLE";
    case TASK_GO_TO_A:  return "GO_TO_A";
    case TASK_GRASP:    return "GRASP";
    case TASK_GO_TO_B:  return "GO_TO_B";
    case TASK_PLACE:    return "PLACE";
    case TASK_DONE:     return "DONE";
    default:            return "???";
    }
}

// ======================== TaskQueue System ======================== 

extern VisualServo_Handle visual_servo;

void TaskQueue_Init(TaskQueue *q)
{
    uint8_t i;
    q->step_count  = 0;
    q->step_index  = 0;
    q->running     = 0;
    q->phase       = TQ_IDLE;
    q->phase_start_tick = 0;
    q->check_tick  = 0;
    q->timeout_ticks = 0;
    for (i = 0; i < 4; i++)
        q->enc_start[i] = 0;
}

int TaskQueue_AddStep(TaskQueue *q, const TaskStep *step)
{
    if (q->step_count >= TASK_QUEUE_MAX)
        return 0;
    q->steps[q->step_count] = *step;
    q->step_count++;
    return 1;
}

void TaskQueue_Start(TaskQueue *q)
{
    uint8_t i;
    if (q->step_count == 0)
        return;
    if (q->running)
    {
        printf("TaskQueue: already running, stop first\r\n");
        return;
    }
    q->step_index = 0;
    q->running    = 1;
    q->phase      = TQ_TRAVEL;
    q->phase_start_tick = g_sys_tick;
    q->check_tick = g_sys_tick;
    // 超时: 每步默认 30 秒 = 3000 ticks 
    q->timeout_ticks = 3000;

    // 快照编码器起始位置 
    for (i = 0; i < 4; i++)
        q->enc_start[i] = Encoder_GetPosition(i);

    // 根据第一步的 travel_mode 设置工作模式 
    if (q->steps[0].travel_mode != 0)
        work_mode = MODE_LINE_FOLLOW;
    else
        work_mode = MODE_MANUAL;

    printf("TaskQueue: started, %d steps\r\n", q->step_count);
}

void TaskQueue_Stop(TaskQueue *q)
{
    uint8_t i;
    q->running = 0;
    q->phase   = TQ_IDLE;
    work_mode  = MODE_MANUAL;
    for (i = 0; i < 4; i++)
        target_speed[i] = 0;
    printf("TaskQueue: stopped\r\n");
}

TaskQueueState TaskQueue_GetState(const TaskQueue *q)
{
    return q->phase;
}

int TaskQueue_IsRunning(const TaskQueue *q)
{
    return (q->running != 0) ? 1 : 0;
}

const char* TaskQueue_StateName(TaskQueueState s)
{
    switch (s) {
    case TQ_IDLE:   return "IDLE";
    case TQ_TRAVEL: return "TRAVEL";
    case TQ_DOCK:   return "DOCK";
    case TQ_ACTION: return "ACTION";
    case TQ_DONE:   return "DONE";
    case TQ_ERROR:  return "ERROR";
    default:        return "???";
    }
}

void TaskQueue_Process(TaskQueue *q)
{
    TaskStep *step;
    int32_t avg_dist;
    uint8_t i;

    if (!q->running)
        return;

    if (q->step_index >= q->step_count)
    {
        q->running = 0;
        q->phase   = TQ_DONE;
        work_mode  = MODE_MANUAL;
        for (i = 0; i < 4; i++)
            target_speed[i] = 0;
        printf("TaskQueue: all steps done!\r\n");
        return;
    }

    step = &q->steps[q->step_index];

    // 超时检测 
    if (q->phase != TQ_IDLE && q->phase != TQ_DONE && q->phase != TQ_ERROR)
    {
        if (g_sys_tick - q->phase_start_tick > q->timeout_ticks)
        {
            q->phase = TQ_ERROR;
            work_mode = MODE_MANUAL;
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            printf("TaskQueue: timeout at step %d\r\n", q->step_index);
            return;
        }
    }

    switch (q->phase)
    {

    case TQ_TRAVEL:
    {
        // 每 50ms 检查一次 
        if (g_sys_tick - q->check_tick < 5)
            return;
        q->check_tick = g_sys_tick;

        avg_dist = (
            (Encoder_GetPosition(0) - q->enc_start[0]) +
            (Encoder_GetPosition(1) - q->enc_start[1]) +
            (Encoder_GetPosition(2) - q->enc_start[2])
        ) / 3;

        // 触发条件 1: 编码器里程到达 
        if (avg_dist >= step->encoder_dist)
        {
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            work_mode = MODE_MANUAL;
            goto enter_dock;
        }

        // 触发条件 2: 巡线过程中提前检测到目标标签 (自动切入视觉伺服) 
        // 使用 OpenMV_TagSeenRecently 而非 OpenMV_GetData, 避免与 ISR 抢数据 
        if (step->tag_id >= 0)
        {
            int16_t seen_tag;
            seen_tag = OpenMV_GetLastTagId();
            if (seen_tag >= 0 && (step->tag_id < 0 || seen_tag == step->tag_id))
            {
                // 标签在最近 100ms (10 ticks) 内出现过 → 判断为已进入对接范围 
                if (OpenMV_TagSeenRecently(step->tag_id, 10))
                {
                    for (i = 0; i < 4; i++)
                        target_speed[i] = 0;
                    work_mode = MODE_MANUAL;
                    printf("TaskQueue: tag %d spotted, docking now\r\n", seen_tag);
                    goto enter_dock;
                }
            }
        }
        return;

enter_dock:
        // 判断是否需要视觉对接 
        if (step->tag_id >= 0)
        {
            // 设置视觉伺服目标 
            VisualServo_SetTarget(&visual_servo,
                step->tag_id, step->dock_distance);
            VisualServo_Reset(&visual_servo);
            work_mode = MODE_VISUAL_SERVO;
            q->phase  = TQ_DOCK;
            q->phase_start_tick = g_sys_tick;
            // dock 阶段超时 10 秒 
            q->timeout_ticks = 1000;
            printf("TaskQueue: step %d, docking to tag %d\r\n",
                q->step_index, step->tag_id);
        }
        else
        {
            // 无需对接，直接执行动作 
            q->phase  = TQ_ACTION;
            q->phase_start_tick = g_sys_tick;
            q->timeout_ticks = 800;
            printf("TaskQueue: step %d, action (no dock)\r\n",
                q->step_index);
        }
        return;
    }

    case TQ_DOCK:
    {
        // 检查是否对准 
        if (VisualServo_IsAligned(&visual_servo))
        {
            work_mode = MODE_MANUAL;
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            q->phase  = TQ_ACTION;
            q->phase_start_tick = g_sys_tick;
            q->timeout_ticks = 800;
            printf("TaskQueue: step %d, docked!\r\n", q->step_index);
        }
        else if (VisualServo_IsLost(&visual_servo))
        {
            // 标签持续丢失 
            q->phase = TQ_ERROR;
            work_mode = MODE_MANUAL;
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            printf("TaskQueue: step %d, tag lost!\r\n", q->step_index);
        }
        return;
    }

    case TQ_ACTION:
    {
        // 执行动作 (阻塞, 在主循环中安全) 
        switch (step->action)
        {
        case ACTION_GRASP:
            Action_Grasp();
            ArmSM_NotifyComplete();
            break;
        case ACTION_PLACE:
            Action_Place(step->action_param);
            ArmSM_NotifyComplete();
            break;
        case ACTION_HOOK:
            Action_HookTrailer();
            break;
        case ACTION_UNHOOK:
            Action_UnhookTrailer();
            break;
        default:
            // ACTION_NONE: 无动作, 仅停靠 
            break;
        }

        printf("TaskQueue: step %d action done\r\n", q->step_index);

        // 推进到下一步 
        q->step_index++;
        if (q->step_index >= q->step_count)
        {
            q->running = 0;
            q->phase   = TQ_DONE;
            work_mode  = MODE_MANUAL;
            for (i = 0; i < 4; i++)
                target_speed[i] = 0;
            printf("TaskQueue: mission complete!\r\n");
            return;
        }

        // 准备下一步的 TRAVEL 阶段 
        for (i = 0; i < 4; i++)
            q->enc_start[i] = Encoder_GetPosition(i);

        step = &q->steps[q->step_index];
        if (step->travel_mode != 0)
            work_mode = MODE_LINE_FOLLOW;
        else
            work_mode = MODE_MANUAL;

        q->phase  = TQ_TRAVEL;
        q->phase_start_tick = g_sys_tick;
        q->check_tick = g_sys_tick;
        q->timeout_ticks = 3000;
        printf("TaskQueue: step %d, traveling %ld counts\r\n",
            q->step_index, (long)step->encoder_dist);
        return;
    }

    default:
        return;
    }
}
