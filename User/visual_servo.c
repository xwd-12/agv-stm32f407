#include "stm32f4xx.h"
#include "visual_servo.h"

/**
 * @brief  初始化视觉伺服句柄
 */
void VisualServo_Init(VisualServo_Handle *vs,
    int16_t target_tag_id, int16_t target_dist_cm,
    int16_t base_speed, float kp_lat, float ki_lat, float kd_lat,
    float kp_long, float ki_long, float kd_long, float dt)
{
    vs->target_tag_id      = target_tag_id;
    vs->target_distance_cm = target_dist_cm;
    vs->target_cx          = 160;       /* 320x240 图像的水平中心 */
    vs->base_speed_max     = base_speed;
    vs->turn_limit         = 200;       /* 差速限幅 (倒车时转弯不能太猛, 否则甩头丢码) */

    /* 横向 PID */
    vs->kp_lat = kp_lat;
    vs->ki_lat = ki_lat;
    vs->kd_lat = kd_lat;
    vs->integal_lat   = 0.0f;
    vs->last_err_lat  = 0.0f;
    vs->last_deriv_lat = 0.0f;

    /* 纵向 PID */
    vs->kp_long = kp_long;
    vs->ki_long = ki_long;
    vs->kd_long = kd_long;
    vs->integal_long   = 0.0f;
    vs->last_err_long  = 0.0f;
    vs->last_deriv_long = 0.0f;

    /* 通用 */
    vs->integral_sep_th = 60.0f;        /* 横向像素误差 > 60 时积分清零 */
    vs->dt = dt;

    /* 对准判定 */
    vs->aligned              = 0;
    vs->lost_cnt             = 0;
    vs->lost_threshold       = 20;      /* 20 帧 = 200ms 无标签判定丢失 */
    vs->aligned_threshold_x  = 10;      /* 横向 ±10 像素内视作对准 */
    vs->aligned_threshold_d  = 3;       /* 纵向 ±3cm 内视作对准 */
    vs->aligned_frames       = 0;
    vs->aligned_required     = 5;       /* 连续 5 帧对准判定稳定 */
}

/**
 * @brief  设置目标标签和距离
 */
void VisualServo_SetTarget(VisualServo_Handle *vs,
    int16_t tag_id, int16_t dist_cm)
{
    vs->target_tag_id      = tag_id;
    vs->target_distance_cm = dist_cm;
    vs->aligned            = 0;
    vs->aligned_frames     = 0;
}

/**
 * @brief  设置横向 PID 参数
 */
void VisualServo_SetLatPID(VisualServo_Handle *vs,
    float kp, float ki, float kd)
{
    vs->kp_lat = kp;
    vs->ki_lat = ki;
    vs->kd_lat = kd;
}

/**
 * @brief  设置纵向 PID 参数
 */
void VisualServo_SetLongPID(VisualServo_Handle *vs,
    float kp, float ki, float kd)
{
    vs->kp_long = kp;
    vs->ki_long = ki;
    vs->kd_long = kd;
}

/**
 * @brief  设置对准阈值
 */
void VisualServo_SetAlignThreshold(VisualServo_Handle *vs,
    int16_t x_px, int16_t d_cm, uint8_t frames)
{
    vs->aligned_threshold_x = x_px;
    vs->aligned_threshold_d = d_cm;
    vs->aligned_required    = frames;
}

/**
 * @brief  设置丢失阈值 (连续多少帧无标签判定丢失)
 */
void VisualServo_SetLostThreshold(VisualServo_Handle *vs,
    uint8_t threshold)
{
    vs->lost_threshold = threshold;
}

/**
 * @brief  重置 PID 积分和微分状态
 */
void VisualServo_Reset(VisualServo_Handle *vs)
{
    vs->integal_lat    = 0.0f;
    vs->last_err_lat   = 0.0f;
    vs->last_deriv_lat = 0.0f;
    vs->integal_long    = 0.0f;
    vs->last_err_long   = 0.0f;
    vs->last_deriv_long = 0.0f;
    vs->aligned         = 0;
    vs->aligned_frames  = 0;
    vs->lost_cnt        = 0;
}

/**
 * @brief  查询是否对准
 */
int VisualServo_IsAligned(const VisualServo_Handle *vs)
{
    return (vs->aligned != 0) ? 1 : 0;
}

/**
 * @brief  查询是否丢失标签
 */
int VisualServo_IsLost(const VisualServo_Handle *vs)
{
    return (vs->lost_cnt > vs->lost_threshold) ? 1 : 0;
}

/**
 * @brief  视觉伺服主任务 (ISR 中 100Hz 调用, 仿 LineFollow_Task 模式)
 *
 *         omv == NULL 表示本周期无新数据，仅更新丢失计数
 *         omv->tag_id == -1 表示 OpenMV 未检测到任何标签
 *         横向 PID 控制转向 (turn)，纵向 PID 控制前进/后退速度
 *         差速合成: left = forward + turn, right = forward - turn
 */
void VisualServo_Task(VisualServo_Handle *vs,
    const OpenMV_Data *omv, volatile int16_t target_speed[4])
{
    float error_lat;
    float error_long;
    float derivative;
    float turn;
    float forward;
    int16_t left;
    int16_t right;

    /* 无新数据：增加丢失计数，保持当前速度不变 */
    if (omv == ((const OpenMV_Data *)0))
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        return;
    }

    /* 标签丢失 (OpenMV 未检测到任何标签) */
    if (omv->tag_id < 0)
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        /* 连续丢失超过阈值：减速停止 */
        if (vs->lost_cnt > vs->lost_threshold)
        {
            target_speed[0] = 0;
            target_speed[1] = 0;
            target_speed[2] = 0;
            target_speed[3] = 0;
            vs->aligned = 0;
            vs->aligned_frames = 0;
        }
        return;
    }

    /* 标签 ID 不匹配 (指定了目标标签且当前标签不是目标) */
    if (vs->target_tag_id >= 0 && omv->tag_id != vs->target_tag_id)
    {
        /* Bugfix H12: 看到错误标签时逐渐减速, 防止全速漂移 */
        {
            int di;
            for (di = 0; di < 4; di++) {
                int16_t sp = target_speed[di];
                if (sp > 50)  target_speed[di] = sp - 50;
                else if (sp < -50) target_speed[di] = sp + 50;
                else target_speed[di] = 0;
            }
        }
        return;
    }

    /* 有效标签：清零丢失计数 */
    vs->lost_cnt = 0;

    /* ===== 横向 PID (X 中心对准 → 转向控制) ===== */
    error_lat = (float)(vs->target_cx - omv->cx);

    /* 积分分离 */
    if (error_lat > vs->integral_sep_th || error_lat < -vs->integral_sep_th)
        vs->integal_lat = 0.0f;

    /* 梯形积分 */
    vs->integal_lat += (error_lat + vs->last_err_lat) * 0.5f * vs->dt;
    if (vs->integal_lat > 200.0f)  vs->integal_lat = 200.0f;
    if (vs->integal_lat < -200.0f) vs->integal_lat = -200.0f;

    /* 微分 + IIR 低通滤波 */
    derivative = (error_lat - vs->last_err_lat) / vs->dt;
    vs->last_deriv_lat = 0.3f * vs->last_deriv_lat + 0.7f * derivative;
    derivative = vs->last_deriv_lat;

    /* 横向 PID 输出 */
    turn = vs->kp_lat * error_lat + vs->ki_lat * vs->integal_lat
           + vs->kd_lat * derivative;
    vs->last_err_lat = error_lat;

    /* 限幅 */
    if (turn > (float)vs->turn_limit)  turn = (float)vs->turn_limit;
    if (turn < (float)(-vs->turn_limit)) turn = (float)(-vs->turn_limit);

    /* ===== 纵向 PID (距离控制 → 前进/后退速度) ===== */
    error_long = (float)(vs->target_distance_cm - omv->distance_cm);

    /* 积分分离: 距离误差 > 30cm 时清零积分 */
    if (error_long > 30.0f || error_long < -30.0f)
        vs->integal_long = 0.0f;

    /* 梯形积分 */
    vs->integal_long += (error_long + vs->last_err_long) * 0.5f * vs->dt;
    if (vs->integal_long > 200.0f)  vs->integal_long = 200.0f;
    if (vs->integal_long < -200.0f) vs->integal_long = -200.0f;

    /* 微分 + IIR 低通滤波 */
    derivative = (error_long - vs->last_err_long) / vs->dt;
    vs->last_deriv_long = 0.3f * vs->last_deriv_long + 0.7f * derivative;
    derivative = vs->last_deriv_long;

    /* 纵向 PID 输出 */
    forward = vs->kp_long * error_long + vs->ki_long * vs->integal_long
              + vs->kd_long * derivative;
    vs->last_err_long = error_long;

    /* 限幅 */
    if (forward > (float)vs->base_speed_max)  forward = (float)vs->base_speed_max;
    if (forward < (float)(-vs->base_speed_max)) forward = (float)(-vs->base_speed_max);

    /* ===== 差速合成 ===== */
    left  = (int16_t)(forward + turn);
    right = (int16_t)(forward - turn);

    /* 限幅 ±1000 */
    if (left > 1000)   left = 1000;
    if (left < -1000)  left = -1000;
    if (right > 1000)  right = 1000;
    if (right < -1000) right = -1000;

    target_speed[0] = left;
    target_speed[1] = right;
    target_speed[2] = left;
    target_speed[3] = right;

    /* ===== 对准判定 ===== */
    {
        float abs_lat;
        float abs_long;
        if (error_lat < 0.0f) abs_lat = -error_lat; else abs_lat = error_lat;
        if (error_long < 0.0f) abs_long = -error_long; else abs_long = error_long;

        if (abs_lat < (float)vs->aligned_threshold_x &&
            abs_long < (float)vs->aligned_threshold_d)
        {
            if (vs->aligned_frames < 255)
                vs->aligned_frames++;
            if (vs->aligned_frames >= vs->aligned_required)
                vs->aligned = 1;
        }
        else
        {
            vs->aligned_frames = 0;
            vs->aligned = 0;
        }
    }
}

/**
 * @brief  视觉伺服倒车版 (从车对接用)
 *
 *         与 VisualServo_Task 相同的双 PID 计算,
 *         但输出取反: forward -> -forward (倒车靠近),
 *         turn 也取反 (倒车时转向镜像).
 *         复用同一套 PID 参数, 方便 LLM tuner 统一调参.
 */
void VisualServo_TaskReverse(VisualServo_Handle *vs,
    const OpenMV_Data *omv, volatile int16_t target_speed[4])
{
    float error_lat;
    float error_long;
    float derivative;
    float turn;
    float forward;
    int16_t left;
    int16_t right;

    /* 无新数据: 保持当前速度 */
    if (omv == ((const OpenMV_Data *)0))
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        return;
    }

    /* 标签丢失 */
    if (omv->tag_id < 0)
    {
        if (vs->lost_cnt < 255)
            vs->lost_cnt++;
        if (vs->lost_cnt > vs->lost_threshold)
        {
            target_speed[0] = 0;
            target_speed[1] = 0;
            target_speed[2] = 0;
            target_speed[3] = 0;
            vs->aligned = 0;
            vs->aligned_frames = 0;
        }
        return;
    }

    /* 标签 ID 不匹配 */
    if (vs->target_tag_id >= 0 && omv->tag_id != vs->target_tag_id)
    {
        int di;
        for (di = 0; di < 4; di++) {
            int16_t sp = target_speed[di];
            if (sp > 50)  target_speed[di] = sp - 50;
            else if (sp < -50) target_speed[di] = sp + 50;
            else target_speed[di] = 0;
        }
        return;
    }

    vs->lost_cnt = 0;

    /* ===== 横向 PID (同正向版) ===== */
    error_lat = (float)(vs->target_cx - omv->cx);
    if (error_lat > vs->integral_sep_th || error_lat < -vs->integral_sep_th)
        vs->integal_lat = 0.0f;
    vs->integal_lat += (error_lat + vs->last_err_lat) * 0.5f * vs->dt;
    if (vs->integal_lat > 200.0f)  vs->integal_lat = 200.0f;
    if (vs->integal_lat < -200.0f) vs->integal_lat = -200.0f;
    derivative = (error_lat - vs->last_err_lat) / vs->dt;
    vs->last_deriv_lat = 0.3f * vs->last_deriv_lat + 0.7f * derivative;
    derivative = vs->last_deriv_lat;
    turn = vs->kp_lat * error_lat + vs->ki_lat * vs->integal_lat
           + vs->kd_lat * derivative;
    vs->last_err_lat = error_lat;
    /* 横向死区: 偏差<3px 不修正. 原来±8px 太宽 → 车几乎全程直线后退只有腰座在动;
       现在小偏差也会差速转向, 轮子做看得见的对准, 车灵活很多 */
    if (error_lat > -3.0f && error_lat < 3.0f)
        turn = 0.0f;

    /* ===== 纵向 PID (同正向版) ===== */
    error_long = (float)(vs->target_distance_cm - omv->distance_cm);
    if (error_long > 30.0f || error_long < -30.0f)
        vs->integal_long = 0.0f;
    vs->integal_long += (error_long + vs->last_err_long) * 0.5f * vs->dt;
    if (vs->integal_long > 200.0f)  vs->integal_long = 200.0f;
    if (vs->integal_long < -200.0f) vs->integal_long = -200.0f;
    derivative = (error_long - vs->last_err_long) / vs->dt;
    vs->last_deriv_long = 0.3f * vs->last_deriv_long + 0.7f * derivative;
    derivative = vs->last_deriv_long;
    forward = vs->kp_long * error_long + vs->ki_long * vs->integal_long
              + vs->kd_long * derivative;
    vs->last_err_long = error_long;
    /* 倒车对接单向: forward 只能 ≤0 (后退/停), 未到 OK 绝不允许前进 */
    if (forward > 0.0f) forward = 0.0f;
    /* ===== 倒车速度曲线: 线性减速逼近 (消除原断崖跳变) =====
       原逻辑: error_long<-3 (dist>38) 强制全速 -160, dist≤38 只剩 -4.5,
       38↔39cm 之间速度断崖 -160→-4.5, 车"突进→停→突进→停", dist 卡 37-40 压不到.
       新曲线 (target 由 VisualServo_Init 设置, 当前 40cm, base=160):
         dist ≥ target+5 (err ≤ -5)   → 全速 -base, 快速接近
         target < dist < target+5 (-5<err<0) → 线性 -base → 0, 保底 -70 保证推进力
         dist ≤ target (err ≥ 0)    → 0, 到位
       保底 -70: 实测降速段(40-42cm)速度被 turn 分走后右轮只剩 -20 过不了
       电机死区, 车"跑不动" dist 卡住; 保底提到 -70 保证两轮都有驱动力
       实测: 44cm→-128, 43cm→-96, 42cm→-70, 41cm→-70, 平滑压到 40 */
    if (error_long <= -5.0f) {
        forward = -(float)vs->base_speed_max;
    } else if (error_long < 0.0f) {
        forward = -(float)vs->base_speed_max * (-error_long / 5.0f);
        /* 保底倒车速度: 接近时不降为 0 (否则被 turn 分走后过不了电机死区) */
        if (forward > -70.0f) forward = -70.0f;
    } else {
        forward = 0.0f;
    }
    /* 码偏大时不再减速/停车: 转向靠 kp + 倒车安全钳制 (见下) */

    /* turn 限幅:
       原地转(forward≈0): turn 到 120 过电机死区, 快速调角度(否则转不动)
       倒车(forward<0):   限幅占比随速度变化: 全速(-140)给到 40% (=50),
       低速(近距离 -112)给到 ~50% — 实测低速 30% 限幅转向太弱,
       码会从画面中心一路滑到 cx≈60, 车斜着怼过去 (OK 后前进即甩丢码) */
    {
        float turn_max;
        float fwd_abs;
        float ratio;
        fwd_abs = (forward < 0.0f) ? -forward : forward;
        if (fwd_abs < 1.0f) {
            /* 原地转向: 必须够大过死区, 否则 cx 卡住转不动 */
            turn_max = 120.0f;
        } else {
            ratio = 0.75f - 0.35f * (fwd_abs / (float)vs->base_speed_max);
            turn_max = fwd_abs * ratio;
            if (turn_max > (float)vs->turn_limit * 0.25f)
                turn_max = (float)vs->turn_limit * 0.25f;
            if (turn_max < 8.0f) turn_max = 8.0f;
        }
        if (turn > turn_max) turn = turn_max;
        if (turn < -turn_max) turn = -turn_max;
    }

    /* ===== 倒车安全钳制: 倒车对接中任何轮子不得正转(前进) =====
       right = forward + turn, left = forward - turn, 需均 ≤ 0 → |turn| ≤ |forward|
       倒车后期 forward 很小(如 -4.5) 而转向需求大(如 45px→turn=45) 时,
       若让 turn 超过 |forward|, right = forward+turn > 0 → 轮子正转, 车斜着前拱
       (实测: dist 卡 36-38cm 进不去, cx 停在 113-119 修不动).
       解法: 转向需求 > 倒车速度 → forward 归零原地转向(车不位移, 修好 cx 再倒),
       转向需求 ≤ 倒车速度 → 正常倒车+转向, 两轮都保持 ≤0 */
    if (forward < 0.0f) {
        float fwd_abs = -forward;
        if (turn > fwd_abs) {
            /* 需要更强转向: 停车原地转, 转向输出过死区 */
            forward = 0.0f;
            if (turn > 120.0f)  turn = 120.0f;
            if (turn < -120.0f) turn = -120.0f;
        } else if (turn < -fwd_abs) {
            forward = 0.0f;
            if (turn > 120.0f)  turn = 120.0f;
            if (turn < -120.0f) turn = -120.0f;
        }
    }

    /* ===== 倒车输出: forward 负=倒车靠近;
       差速合成(倒车时转向镜像): 码在画面右(cx>160)=车左侧 → 车尾向左摆
       → 右轮向后更快(right更负) → turn<0 需 right 更负 → left=forward-turn, right=forward+turn ===== */
    left  = (int16_t)(forward - turn);
    right = (int16_t)(forward + turn);

    if (left > 1000)   left = 1000;
    if (left < -1000)  left = -1000;
    if (right > 1000)  right = 1000;
    if (right < -1000) right = -1000;

    target_speed[0] = left;
    target_speed[1] = right;
    target_speed[2] = left;
    target_speed[3] = right;

    /* ===== 对准判定 (同正向版) ===== */
    {
        float abs_lat;
        float abs_long;
        if (error_lat < 0.0f) abs_lat = -error_lat; else abs_lat = error_lat;
        if (error_long < 0.0f) abs_long = -error_long; else abs_long = error_long;

        if (abs_lat < (float)vs->aligned_threshold_x &&
            abs_long < (float)vs->aligned_threshold_d)
        {
            if (vs->aligned_frames < 255)
                vs->aligned_frames++;
            if (vs->aligned_frames >= vs->aligned_required)
                vs->aligned = 1;
        }
        else
        {
            vs->aligned_frames = 0;
            vs->aligned = 0;
        }
    }
}
