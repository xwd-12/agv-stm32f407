#include "stm32f4xx.h"
#include "line_sensor.h"
#include <stdbool.h>

// Standard 5-channel line sensor weights (L->R): LEFT1=-2, LEFT2=-1, CENTER=+1, RIGHT2=+2
// Normal driving: center sensor on line, error=0 → error=0
// All 5 channels debounced (3 samples, 30ms) to filter flicker
static const int8_t weights[5] = { 2, 1, 0, -1, -2 };

void Line_sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);

    // All sensor pins: input, pull-up
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

    // Init in physical order: LEFT1 -> LEFT2 -> CENTER -> RIGHT2 (4ch, PA4 unused)
    GPIO_InitStruct.GPIO_Pin = SENSOR_PIN_LEFT1;
    GPIO_Init(SENSOR_PORT_LEFT1, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SENSOR_PIN_LEFT2;
    GPIO_Init(SENSOR_PORT_LEFT2, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SENSOR_PIN_CENTER;
    GPIO_Init(SENSOR_PORT_CENTER, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SENSOR_PIN_RIGHT2;
    GPIO_Init(SENSOR_PORT_RIGHT2, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SENSOR_PIN_RIGHT1;
    GPIO_Init(SENSOR_PORT_RIGHT1, &GPIO_InitStruct);
}

// Read 5-channel sensor states (active-high: 1 = line detected)
// states[0]=RIGHT1(PA4), states[1]=RIGHT2(PC3), states[2]=CENTER(PC2), states[3]=LEFT2(PC1), states[4]=LEFT1(PC0)
// All channels debounced: 3 consecutive same readings (30ms @ 100Hz)
void LineSensor_Read(bool states[5])
{
    bool raw[5];
    static bool stable[5] = {0, 0, 0, 0, 0};
    static uint8_t db_cnt[5] = {0, 0, 0, 0, 0};
    static bool prev_raw[5] = {0, 0, 0, 0, 0};
    uint8_t ch;

    raw[0] = (GPIO_ReadInputDataBit(SENSOR_PORT_RIGHT1, SENSOR_PIN_RIGHT1) != 0);
    raw[1] = (GPIO_ReadInputDataBit(SENSOR_PORT_RIGHT2, SENSOR_PIN_RIGHT2) != 0);
    raw[2] = (GPIO_ReadInputDataBit(SENSOR_PORT_CENTER, SENSOR_PIN_CENTER) != 0);
    raw[3] = (GPIO_ReadInputDataBit(SENSOR_PORT_LEFT2,  SENSOR_PIN_LEFT2) != 0);
    raw[4] = (GPIO_ReadInputDataBit(SENSOR_PORT_LEFT1,  SENSOR_PIN_LEFT1) != 0);

    for (ch = 0; ch < 5; ch++)
    {
        if (raw[ch] == prev_raw[ch]) {
            db_cnt[ch]++;
            if (db_cnt[ch] >= 3) {
                stable[ch] = raw[ch];
                db_cnt[ch] = 3;
            }
        } else {
            db_cnt[ch] = 0;
        }
        prev_raw[ch] = raw[ch];
        states[ch] = stable[ch];
    }
}

// Calculate weighted error from pre-read states
float LineSensor_CalcError(const bool states[5])
{
    int i;
    int err = 0;
    uint16_t detected = 0;

    for (i = 0; i < 5; i++)
    {
        if (states[i] == 1)
        {
            err += weights[i];
            detected++;
        }
    }
    if (detected == 0) return 0.0f;
    return (float)err / (float)detected;
}
