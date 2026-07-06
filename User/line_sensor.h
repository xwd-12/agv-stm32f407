#ifndef  __line_sensor_h
#define  __line_sensor_h
#include "stm32f4xx.h"
#include <stdbool.h>

// 4-channel line-following sensor array on PC0, PC1, PC2, PC3
// PA4 = far right sensor (5ch)
// Normal driving: one of the middle two (LEFT2/PC1 or CENTER/PC2) sees the line
// All 5 channels have 3-sample debounce (30ms) in LineSensor_Read to filter flicker

// Pin definitions in physical order (left to right):
#define SENSOR_PIN_LEFT1     GPIO_Pin_0   // PC0 - far left
#define SENSOR_PORT_LEFT1    GPIOC

#define SENSOR_PIN_LEFT2     GPIO_Pin_1   // PC1 - left middle
#define SENSOR_PORT_LEFT2    GPIOC

#define SENSOR_PIN_CENTER    GPIO_Pin_2   // PC2 - center
#define SENSOR_PORT_CENTER   GPIOC

#define SENSOR_PIN_RIGHT2    GPIO_Pin_3   // PC3 - right middle
#define SENSOR_PORT_RIGHT2   GPIOC

#define SENSOR_PIN_RIGHT1     GPIO_Pin_4   // PA4 - far right
#define SENSOR_PORT_RIGHT1    GPIOA

void Line_sensor_Init(void);
void LineSensor_Read(bool states[5]);     // read 5ch (active-low, 1=line)
float LineSensor_CalcError(const bool states[5]); // 5ch weighted error

#endif
