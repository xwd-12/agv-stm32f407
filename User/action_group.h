#include "stm32f4xx.h"
#ifndef  __action_group_h
#define  __action_group_h

/* =====  (a_set / t_set ) ===== */
/*  */
#define ACT_APPROACH_WAIST     0
#define ACT_APPROACH_SHOULDER  1
#define ACT_APPROACH_ELBOW     2
#define ACT_GRIPPER_CLOSE      3
#define ACT_LIFT_SHOULDER      4
#define ACT_PLACE_WAIST        5
#define ACT_PLACE_SHOULDER     6
#define ACT_PLACE_ELBOW        7
#define ACT_GRIPPER_OPEN       8
#define ACT_RESET_SHOULDER     9
#define ACT_RESET_ELBOW       10
#define ACT_PLACE_OPPOSITE    11
#define ACT_ANGLE_COUNT       12

/*  */
#define ACT_TIME_APPROACH      0
#define ACT_TIME_GRIP          1
#define ACT_TIME_LIFT          2
#define ACT_TIME_PLACE         3
#define ACT_TIME_RETRACT       4
#define ACT_TIME_PLACE_WAIST   5
#define ACT_TIME_COUNT         6

/*  */
extern uint16_t act_angle[ACT_ANGLE_COUNT];
extern uint16_t act_time[ACT_TIME_COUNT];
extern uint8_t  g_skip_waist_home;  /* STATION_MODE:  */

/*  */
void Action_ParamInit(void);

/* / */
void Action_SetAngle(uint8_t id, uint16_t value);
void Action_SetTime(uint8_t id, uint16_t value);
void Action_ShowParams(void);

/*  () */
void Action_Reset(void);
uint8_t Action_ISdle(void);
void Action_Init(void);
void Action_Grasp(void);
void Action_Place(uint16_t waist_angle);
void Action_GraspAndPlace(void);
void Action_HookTrailer(void);
void Action_UnhookTrailer(void);
void Action_Observe(void);
void Action_GraspFromTrailer(void);
void Action_PlaceOnTrailer(void);  /* Station 1:  (30/170 -> 110/100) */
#endif
