#ifndef __arm_config_h
#define __arm_config_h

// ===== 机械臂复位角度(上电默认位置) =====
#define ARM_HOME_WAIST      0
#define ARM_HOME_SHOULDER  60
#define ARM_HOME_ELBOW     25
// 夹爪卡死, 此处保留备用
#define ARM_HOME_GRIPPER     0    /* 0=闭合, 45=张开 */

// 挂钩舵机 (逻辑ID=4, PA10 TIM1_CH3)
#define ARM_HOME_HOOK       90    /* 脱钩/归零=90° */
#define ARM_HOOK_LOCK        0    /* 锁定=0° 钩子卡入从车槽口 */

#endif
