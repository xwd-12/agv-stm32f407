#ifndef  __state_machine_h
#define  __state_machine_h
#include "stm32f4xx.h"

// AGV导航任务状态枚举
typedef enum {
    TASK_IDLE = 0,
    TASK_GO_TO_A,       // line-follow to point A
    TASK_GRASP,         // arm grasp at A
    TASK_GO_TO_B,       // line-follow to point B
    TASK_PLACE,         // arm place at B
    TASK_DONE           // mission complete
} TaskNavState;

// 机械臂状态枚举
typedef enum {
    ARM_IDLE = 0,       // 空闲
    ARM_GRASPING,       // 抓取中
    ARM_PLACING,        // 投放中
    ARM_RESETTING,      // 复位中
    ARM_ESTOP           // 急停
} ArmState;

void  ArmSM_Init(void);
ArmState ArmSM_GetState(void);
uint8_t ArmSM_RequestGrasp(void);
uint8_t ArmSM_RequestPlace(uint16_t waist_angle);
uint8_t ArmSM_RequestReset(void);
void  ArmSM_EmergencyStop(void);
void  ArmSM_NotifyComplete(void);
void  ArmSM_Tick10ms(void);            // 10ms定时器调用, 超时检测
uint8_t ArmSM_IsBusy(void);
const char* ArmSM_StateName(ArmState s);

// ---- AGV Task Navigation API ----
void  TaskNav_Init(void);
void  TaskNav_Start(int32_t dist_a, int32_t dist_b);
void  TaskNav_Stop(void);
void  TaskNav_Process(void);           // call in main loop (non-blocking except arm actions)
TaskNavState TaskNav_GetState(void);
int32_t TaskNav_GetProgress(void);     // current encoder distance traveled
int32_t TaskNav_GetTarget(void);       // current segment target distance
const char* TaskNav_StateName(TaskNavState s);

// ===== 动作类型 =====
#define ACTION_NONE    0
#define ACTION_GRASP   1
#define ACTION_PLACE   2
#define ACTION_HOOK    3   // 勾从车(预留)
#define ACTION_UNHOOK  4   // 脱从车(预留)

// ===== 任务步骤 =====
typedef struct {
    uint8_t  travel_mode;    // 0=编码器里程, 1=巡线
    int32_t  encoder_dist;   // 里程目标距离(编码器计数)
    int16_t  tag_id;         // AprilTag ID (-1=不停靠)
    int16_t  dock_distance;  // 视觉对接停靠距离(cm)
    uint8_t  action;         // ACTION_* 动作
    int16_t  action_param;   // 动作参数(如放置角度)
} TaskStep;

// ===== 任务队列(最多16步) =====
#define TASK_QUEUE_MAX  16

typedef enum {
    TQ_IDLE = 0,
    TQ_TRAVEL,          // 巡线/里程走向工位
    TQ_DOCK,            // 视觉对接
    TQ_ACTION,          // 执行动作
    TQ_DONE,
    TQ_ERROR            // 超时/丢失
} TaskQueueState;

typedef struct {
    TaskStep      steps[TASK_QUEUE_MAX];
    uint8_t       step_count;
    uint8_t       step_index;
    uint8_t       running;
    TaskQueueState phase;
    uint32_t      phase_start_tick;
    int32_t       enc_start[4];
    uint32_t      check_tick;
    uint32_t      timeout_ticks;
} TaskQueue;

// ===== TaskQueue API =====
void TaskQueue_Init(TaskQueue *q);
int  TaskQueue_AddStep(TaskQueue *q, const TaskStep *step);
void TaskQueue_Start(TaskQueue *q);
void TaskQueue_Stop(TaskQueue *q);
void TaskQueue_Process(TaskQueue *q);
TaskQueueState TaskQueue_GetState(const TaskQueue *q);
int  TaskQueue_IsRunning(const TaskQueue *q);
const char* TaskQueue_StateName(TaskQueueState s);

#endif
