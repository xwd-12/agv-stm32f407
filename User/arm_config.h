#ifndef __arm_config_h
#define __arm_config_h

// ===== 机械臂复位角度(上电默认位置) =====
#define ARM_HOME_WAIST      5   /* 270°舵机, 0°死区, 改5° */
#define ARM_HOME_SHOULDER    0
#define ARM_HOME_ELBOW     120
// 夹爪
#define ARM_HOME_GRIPPER    50    /* 半开 */

// 挂钩舵机 (逻辑ID=4, PA10 TIM1_CH3)
#define ARM_HOME_HOOK       10    /* 锁定/勾住 */
#define ARM_HOOK_UNLOCK     175   /* 脱钩/向上 (170→175) */

// 从车三个放置位对应的腰座角度 (270°舵机, 正前方=90°, 正后方≈180°)
// 用户指定 2026: 红=185°(195-10) 黄=165°(180-15) 绿=150°
#define TRAILER_POS_RED     185   /* 红色放置位 (195-10) */
#define TRAILER_POS_YELLOW  165   /* 黄色放置位 (180-15) */
#define TRAILER_POS_GREEN   150   /* 绿色放置位 */

// 投放站三个放置位对应的腰座角度 (270°舵机, 正前方=90°)
#define DELIVER_POS_RED      80   /* 投放-红色位置 */
#define DELIVER_POS_YELLOW   90   /* 投放-黄色位置 */
#define DELIVER_POS_GREEN   100   /* 投放-绿色位置 */

// 观测动作 (抓取后展示给摄像头看)
#define OBSERVE_WAIST       180
#define OBSERVE_SHOULDER     80
#define OBSERVE_ELBOW       110

#endif
