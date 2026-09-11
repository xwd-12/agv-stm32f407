#include "stm32f4xx.h"
#include "smooth_servo.h"
#include "delay.h"
#include "arm_config.h"
#include "action_group.h"
#include "uart.h"
#include <stdio.h>

uint8_t g_skip_waist_home = 0;  /* STATION_MODE:  */

/* ===== ( =  #define ) ===== */
uint16_t act_angle[ACT_ANGLE_COUNT] = {
    90,    /* ACT_APPROACH_WAIST     (270) */
   100,    /* ACT_APPROACH_SHOULDER  */
   120,    /* ACT_APPROACH_ELBOW     */
    50,    /* ACT_GRIPPER_CLOSE      */
    88,    /* ACT_LIFT_SHOULDER      */
   165,    /* ACT_PLACE_WAIST        (270) */
    80,    /* ACT_PLACE_SHOULDER     */
   140,    /* ACT_PLACE_ELBOW        */
   170,    /* ACT_GRIPPER_OPEN       */
    90,    /* ACT_RESET_SHOULDER     */
    80,    /* ACT_RESET_ELBOW        */
   240     /* ACT_PLACE_OPPOSITE     (270) */
};

uint16_t act_time[ACT_TIME_COUNT] = {
    4000,  /* ACT_TIME_APPROACH    (3000, ) */
    2000,  /* ACT_TIME_GRIP        (1500) */
    3500,  /* ACT_TIME_LIFT        (2500) */
    4000,  /* ACT_TIME_PLACE       (3000) */
    3000,  /* ACT_TIME_RETRACT     (2000) */
    3000   /* ACT_TIME_PLACE_WAIST (2000) */
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
#define SERVO_HOOK     4   /*  PA10 */

/* Bug#7:  (UARTstop¦Ë) */
static uint8_t g_action_estop = 0;

/**
 * Bug#7: UART¦Ë"stop"
 * , 
 *  1=stop, 0=
 */
static int check_uart_estop(void)
{
    /* ÝR¦Ë "stop", ¦Ê
     * () */
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
    /* , ¦ÄFlash */
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

/*  (Bug#7: UART) */
static void WaitForServo(uint8_t id)
{
    if (g_action_estop) return;
    while (smoothservo_IsBusy(id)) {
        Delay_ms(10);
        if (check_uart_estop()) return;
        Delay_ms(10);
    }
}

/* §Ø (Bug#7: UART) */
static void WaitforAllservo(uint16_t delay_ms)
{
    uint16_t chunk;
    if (g_action_estop) return;
    if (delay_ms > 50) delay_ms = 50;  /* 50msUART */
    while (smoothservo_IsBusy(SERVO_WAIST) ||
           smoothservo_IsBusy(SERVO_SHOULDER) ||
           smoothservo_IsBusy(SERVO_ELBOW) ||
           smoothservo_IsBusy(SERVO_GRIPPER)) {
        /* , §³¦ÊUART */
        for (chunk = 0; chunk < delay_ms; chunk += 10) {
            Delay_ms(10);
            if (check_uart_estop()) return;
        }
    }
}

/* ¦Ë§Û */
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

/*  () */
void Action_Grasp(void)
{
    g_action_estop = 0;  /* Bug#7: ¦Â */

    /* 1.  */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_APPROACH_WAIST], act_time[ACT_TIME_APPROACH]);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */

    /* 2. §³ */
    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_APPROACH_SHOULDER], act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_APPROACH_ELBOW], 1800);
    /*  (Bugfix H11: 0, ) */
    Delay_ms(300);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_OPEN], 800);
    /* §³¦Ë(1.5s) */
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(1200);
    if (g_action_estop) return;  /* Bug#7 */
    /* 3.  */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 1000);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;  /* Bug#7 */
    /* ¦Ë */
    WaitforAllservo(300);
    if (g_action_estop) return;  /* Bug#7 */

    /* 6. (¦Ë) */
    smooth_Settarget(SERVO_SHOULDER, ARM_HOME_SHOULDER, 3000);
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(200);
    smooth_Settarget(SERVO_ELBOW,    ARM_HOME_ELBOW,    3000);
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(800);
    if (g_action_estop) return;  /* Bug#7 */
    /* ¦Ë (STATION_MODE 180) */
    if (!g_skip_waist_home) {
        smooth_Settarget(SERVO_WAIST, ARM_HOME_WAIST, 3000);
        WaitForServo(SERVO_WAIST);
        if (g_action_estop) return;  /* Bug#7 */
        Delay_ms(800);  /* ¦Ë+ */
    }

}

/* : ¦Ë */
void Action_Place(uint16_t waist_angle)
{
    (void)waist_angle;  /* ¨²,  */
    g_action_estop = 0;  /* Bug#7: ¦Â */

    /* 1. () */
    smooth_Settarget(SERVO_WAIST,
        act_angle[ACT_APPROACH_WAIST], act_time[ACT_TIME_APPROACH]);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(800);
    if (g_action_estop) return;  /* Bug#7 */

    /* 2. §³() */
    smooth_Settarget(SERVO_SHOULDER,
        act_angle[ACT_APPROACH_SHOULDER], act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,
        act_angle[ACT_APPROACH_ELBOW], 1800);
    WaitforAllservo(300);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */

    /* 3. ,  */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_OPEN], 1000);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(500);
    if (g_action_estop) return;  /* Bug#7 */

    /* 4. +¦Ë() */
    smooth_Settarget(SERVO_SHOULDER, ARM_HOME_SHOULDER, 3000);
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(200);
    smooth_Settarget(SERVO_ELBOW,    ARM_HOME_ELBOW,    3000);
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(800);
    if (g_action_estop) return;  /* Bug#7 */
    smooth_Settarget(SERVO_WAIST, ARM_HOME_WAIST, 3000);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;  /* Bug#7 */
    Delay_ms(800);  /* ¦Ë+ */

    /* 5. ¦Ë */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 1000);
    WaitForServo(SERVO_GRIPPER);
}

/*  */
void Action_GraspAndPlace(void)
{
    /* ===== 1.  ===== */
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

    /* ===== 2.  ===== */
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

    /* ===== 3. ¦Ë ===== */
    Action_Reset();
}

/* : 90()0() */
void Action_HookTrailer(void)
{
    g_action_estop = 0;
    smooth_Settarget(SERVO_HOOK, ARM_HOME_HOOK, 500);
    WaitForServo(SERVO_HOOK);
    if (g_action_estop) return;
    printf("Trailer hooked\r\n");
}

/* : 0()90() */
void Action_UnhookTrailer(void)
{
    g_action_estop = 0;
    smooth_Settarget(SERVO_HOOK, ARM_HOOK_UNLOCK, 500);
    WaitForServo(SERVO_HOOK);
    if (g_action_estop) return;
    printf("Trailer unhooked\r\n");
}

/* : 180, §³¦Ë,  */
void Action_Observe(void)
{
    g_action_estop = 0;
    smooth_Settarget(SERVO_WAIST,    OBSERVE_WAIST,    1500);
    smooth_Settarget(SERVO_SHOULDER, OBSERVE_SHOULDER, 1500);
    smooth_Settarget(SERVO_ELBOW,    OBSERVE_ELBOW,    1500);
    WaitForServo(SERVO_WAIST);
    if (g_action_estop) return;
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;
    printf("Observe: waist=%d shoulder=%d elbow=%d\r\n",
           OBSERVE_WAIST, OBSERVE_SHOULDER, OBSERVE_ELBOW);
}

/* Station 1 : ¦Ë(shoulder=30, elbow=170) -> ¡¤(110, 100) ->  ->  */
void Action_GraspFromTrailer(void)
{
    g_action_estop = 0;

    /* 1. ¦Ë¡¤: shoulder 30->110, elbow 170->100 */
    smooth_Settarget(SERVO_SHOULDER, 110, act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,    100, 1800);  /* ¦Ë */

    /*  */
    Delay_ms(300);
    if (g_action_estop) return;
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_OPEN], 800);

    /* §³¦Ë,  */
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;
    Delay_ms(1200);
    if (g_action_estop) return;

    /* 2.  */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 1000);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;

    /* ¦Ë */
    WaitforAllservo(300);
    if (g_action_estop) return;

    /* 3. : ¦Ë */
    smooth_Settarget(SERVO_SHOULDER, 30, 3000);
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;
    Delay_ms(800);

    /* :  g_skip_waist_home ,  */
}
/* Station 0 : ¡¤(110/100) ->  ->  */
/* Station 0: lower arm -> OPEN gripper small(140) -> RAISE arm -> CLOSE gripper */
void Action_PlaceOnTrailer(void)
{
    g_action_estop = 0;

    /* 1. lower arm to place pose: shoulder->110, elbow->100, wait until settled */
    smooth_Settarget(SERVO_SHOULDER, 110, act_time[ACT_TIME_APPROACH]);
    smooth_Settarget(SERVO_ELBOW,    100, 1800);
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;
    Delay_ms(300);
    if (g_action_estop) return;

    /* 2. open gripper small(140): just release object, no bounce */
    smooth_Settarget(SERVO_GRIPPER, 140, 800);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;
    Delay_ms(800);
    if (g_action_estop) return;

    /* 3. RAISE arm first (shoulder->30, elbow->170), then close gripper */
    smooth_Settarget(SERVO_SHOULDER, 30, 3000);
    smooth_Settarget(SERVO_ELBOW,    170, 3000);
    WaitForServo(SERVO_SHOULDER);
    if (g_action_estop) return;
    WaitForServo(SERVO_ELBOW);
    if (g_action_estop) return;
    Delay_ms(200);
    if (g_action_estop) return;

    /* 4. close gripper, done */
    smooth_Settarget(SERVO_GRIPPER, act_angle[ACT_GRIPPER_CLOSE], 500);
    WaitForServo(SERVO_GRIPPER);
    if (g_action_estop) return;
    Delay_ms(300);
}

