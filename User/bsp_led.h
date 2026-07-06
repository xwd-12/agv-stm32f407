#ifndef bsp_led_h
#define bsp_led_h
#define LED_ON 0
#define LED_OFF 1
void LED_Init(void);
void LED_Set(int x,int y);//控制1，2引脚灯的熄灭与亮起
void LED1_Turn(void);
void LED2_Turn(void);
void LED_set1(void);
void LED_reset1(void);
void LED_set2(void);
void LED_reset2(void);
void LED_Turn(int x);





#endif
