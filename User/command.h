#ifndef  __command_h
#define  __command_h
#include "stm32f4xx.h"
__weak void Load_parameters(void);
__weak void Save_parameters(void);
void command_init(void);
void Commend_Parse(char*cmd);
void sendPIDDataToUART(void);
int parseAndUpdatePID(const char *cmd);

extern volatile uint32_t g_sys_tick;
extern int16_t last_pid_setpoint[4];
extern int16_t last_pid_speed[4];
extern int16_t last_pid_output[4];
extern int16_t last_pid_error[4];

#endif
