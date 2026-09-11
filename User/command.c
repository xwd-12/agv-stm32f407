#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "motor.h"
#include "servo.h"
#include "pid.h"
#include "enconder.h"
#include "uart.h"
#include "inttypes.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "smooth_servo.h"
#include "state_machine.h"
#include "action_group.h"
#include "OLED.h"
#include "command.h"
#include "Timer.h"
#include "commend_openmv.h"
#include "visual_servo.h"
 extern VisualServo_Handle visual_servo;
 extern TaskQueue task_queue;
 extern volatile uint8_t qr_trig_flag;
 extern volatile uint8_t ai_dbg_stat;
 extern volatile uint8_t ai_dbg_skip;
 extern volatile uint8_t ai_dbg_go;
 extern volatile uint8_t ai_dbg_phase;
extern PID_Handle pid_positon[4];
extern int32_t target_position[4];
extern PID_Handle pid_motor[4];
extern volatile int16_t target_speed[4];
extern int8_t motor_dir[4];
extern int8_t encoder_dir[4];
extern volatile uint8_t motor_enable;
extern volatile uint8_t work_mode;
extern Line_follow_Handle line_follow;
extern int16_t last_pid_setpoint[4];
extern int16_t last_pid_speed[4];
extern int16_t last_pid_output[4];
extern int16_t last_pid_error[4];
extern volatile uint8_t lf_dbg_enable;   /* lfdbg 命令开关 (main.c 定义) */
extern int16_t encoder_max[4];
extern int8_t encoder_dir[4];
extern volatile int32_t line_time_ticks;

void sendPIDDataToUART(void)
{
	int i;
	for (i = 0; i < 4; i++)
	{
		printf("%d,%lu,%d,%d,%d,%d,%.3f,%.3f,%.3f\r\n",
			i,
			g_sys_tick,
			last_pid_setpoint[i],
			last_pid_speed[i],
			last_pid_output[i],
			last_pid_error[i],
			pid_motor[i].Kp,
			pid_motor[i].Ki,
			pid_motor[i].Kd);
	}
}

__weak void Save_parameters(void) {}
__weak void Load_parameters(void) {}

/* stm32f407_openmv profile 遥测: 官方 Status 快照格式 (LLM PID Tuner 用)
 * 注意: tuner 逐行累积, 直到 servo.delta 行触发合并.
 * 需要 target = (x,y) + servo.x + servo.y + servo.delta 全部出现才有效. */
void sendVisualServoTelemetry(void)
{
	extern VisualServo_Handle visual_servo;
	OpenMV_Data omv_telem;
	int16_t cur_cx = 0;
	int16_t cur_dist = 0;

	/* 取最新 OpenMV 数据 (cx=标签中心, dist=距离) */
	if (OpenMV_PeekData(&omv_telem) && omv_telem.tag_id >= 0) {
		cur_cx = omv_telem.cx;
		cur_dist = omv_telem.distance_cm;
	} else {
		cur_cx = 0;
		cur_dist = 0;
	}

	/* Status 快照头 */
	printf("Status:\r\n");
	/* 横向 PID (pid.x) */
	printf("pid.x = %.3f %.3f %.3f\r\n",
		(double)visual_servo.kp_lat,
		(double)visual_servo.ki_lat,
		(double)visual_servo.kd_lat);
	/* 纵向 PID (pid.y) */
	printf("pid.y = %.3f %.3f %.3f\r\n",
		(double)visual_servo.kp_long,
		(double)visual_servo.ki_long,
		(double)visual_servo.kd_long);
	/* 目标坐标: (目标cx, 目标距离) */
	printf("target = (%d,%d)\r\n",
		(int)visual_servo.target_cx,
		(int)visual_servo.target_distance_cm);
	/* 当前位置 */
	printf("servo.x = %d\r\n", (int)cur_cx);
	printf("servo.y = %d\r\n", (int)cur_dist);
	/* 偏差 (触发 tuner 合并快照) */
	printf("servo.delta = %d\r\n",
		(int)(visual_servo.target_cx - cur_cx));
}

void command_init(void)
{
	Load_parameters();
}

int parseAndUpdatePID(const char *cmd)
{
	float p, i, d;
	int motor_id;
	int j;

	if (sscanf(cmd, "SET KP:%f KI:%f KD:%f", &p, &i, &d) == 3 ||
	    sscanf(cmd, "SET P:%f I:%f D:%f", &p, &i, &d) == 3)
	{
		for (j = 0; j < 4; j++)
		{
			pid_motor[j].Kp = p;
			pid_motor[j].Ki = i;
			pid_motor[j].Kd = d;
			}
		printf("PID set: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", p, i, d);
		return 1;
		}

	if (sscanf(cmd, "SET KP:%f KI:%f KD:%f M:%d", &p, &i, &d, &motor_id) == 4 ||
	    sscanf(cmd, "SET P:%f I:%f D:%f M:%d", &p, &i, &d, &motor_id) == 4)
	{
		if (motor_id >= 0 && motor_id < 4)
		{
			pid_motor[motor_id].Kp = p;
			pid_motor[motor_id].Ki = i;
			pid_motor[motor_id].Kd = d;
			printf("Motor[%d] PID: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", motor_id, p, i, d);
			}
		return 1;
		}

	if (sscanf(cmd, "PID %f %f %f", &p, &i, &d) == 3)
	{
		for (j = 0; j < 4; j++)
		{
			pid_motor[j].Kp = p;
			pid_motor[j].Ki = i;
			pid_motor[j].Kd = d;
			}
		printf("PID set: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", p, i, d);
		return 1;
		}

	return 0;
}

void Commend_Parse(char *cmd)
{
	char op[16];
	int motor_id;
	float fval;
	int ival;
	uint8_t servo_id;
	uint16_t angle;
	uint16_t time_ms;
	int i;
	int32_t pos;
	int16_t spd;
	int32_t raw;

	if (parseAndUpdatePID(cmd))
		return;

	else if(sscanf(cmd,"kp %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_motor[motor_id].Kp=fval;
		printf("Motor[%d] Kp=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"ki %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_motor[motor_id].Ki=fval;
		printf("Motor[%d] Ki=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"kd %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_motor[motor_id].Kd=fval;
		printf("Motor[%d] Kd=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"spd %d %d",&motor_id,&spd)==2&&motor_id>=0&&motor_id<4)
	{
		target_speed[motor_id]=spd;
		printf("Speed[%d] = %d\r\n",motor_id,spd);
		}
	else if (sscanf(cmd, "m %c", op) == 1)
	{
		switch (op[0])
		{
		case 'f': for(i=0;i<4;i++)target_speed[i]=300; printf("Forward\r\n"); break;
		case 'b': for(i=0;i<4;i++)target_speed[i]=-300; printf("Backward\r\n"); break;
		case 'l': for(i=0;i<4;i++)target_speed[i]=(i%2==0)?-200:200; printf("Left\r\n"); break;
		case 'r': for(i=0;i<4;i++)target_speed[i]=(i%2==0)?200:-200; printf("Right\r\n"); break;
		case 's': for(i=0;i<4;i++)target_speed[i]=0; printf("Stop\r\n"); break;
			}
		}
	else if(sscanf(cmd,"pos_kp %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_positon[motor_id].Kp=fval;
		printf("Pos[%d] Kp=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"pos_ki %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_positon[motor_id].Ki=fval;
		printf("Pos[%d] Ki=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"pos_kd %d %f",&motor_id,&fval)==2&&motor_id>=0&&motor_id<4)
	{
		pid_positon[motor_id].Kd=fval;
		printf("Pos[%d] Kd=%.3f\r\n",motor_id,fval);
		}
	else if(sscanf(cmd,"pos_target %d %d",&motor_id,&spd)==2&&motor_id>=0&&motor_id<4)
	{
		target_position[motor_id]=(int32_t)spd;
		printf("Pos target[%d] = %ld\r\n",motor_id,(long)target_position[motor_id]);
		}
	else if(sscanf(cmd,"enc_pos %d",&motor_id)==1&&motor_id>=0&&motor_id<4)
		printf("Encoder[%d] pos=%ld\r\n",motor_id,(long)Encoder_GetPosition(motor_id));
	else if(sscanf(cmd,"enc_speed %d",&motor_id)==1&&motor_id>=0&&motor_id<4)
		printf("Encoder[%d] speed=%d\r\n",motor_id,Encoder_getspeed(motor_id));
	else if(sscanf(cmd,"enc_max %d %d",&motor_id,&spd)==2&&motor_id>=0&&motor_id<4)
	{
		encoder_max[motor_id]=spd;
		printf("Encoder max[%d]=%d\r\n",motor_id,spd);
		}
	else if(sscanf(cmd,"enc_dir %d %d",&motor_id,&spd)==2&&motor_id>=0&&motor_id<4)
	{
		encoder_dir[motor_id]=(int8_t)spd;
		printf("Encoder dir[%d]=%d\r\n",motor_id,spd);
		}
	else if(sscanf(cmd,"enc_clear %d",&motor_id)==1&&motor_id>=0&&motor_id<4)
	{
		Encoder_ClearPosition(motor_id);
		printf("Encoder[%d] cleared\r\n",motor_id);
		}
	else if(strcmp(cmd,"enc_speed_all")==0)
	{
		printf("Speeds: ");
		for(i=0;i<4;i++)printf("%d ",Encoder_getspeed(i));
		printf("\r\n");
		}
	else if(sscanf(cmd,"enc_raw %d",&motor_id)==1&&motor_id>=0&&motor_id<4)
		printf("Encoder[%d] raw=%d\r\n",motor_id,Encoder_GetPosition(motor_id));
	else if(strcmp(cmd,"enc_raw")==0)
	{
		for(i=0;i<4;i++)
			printf("Enc[%d] raw=%d\r\n",i,Encoder_GetPosition(i));
		}
	else if(strcmp(cmd,"enc_oled")==0)
		printf("OLED not connected\r\n");
	else if(sscanf(cmd,"pid_dbg %d",&motor_id)==1&&motor_id>=0&&motor_id<4)
		printf("Motor[%d] sp=%d real=%d out=%d err=%d\r\n",
			motor_id,last_pid_setpoint[motor_id],last_pid_speed[motor_id],
			last_pid_output[motor_id],last_pid_error[motor_id]);
	else if(sscanf(cmd,"base_speed %d",&spd)==1)
	{
		line_follow.base_speed=spd;
		printf("Base speed=%d\r\n",spd);
		}
	else if(sscanf(cmd,"lkp %f",&fval)==1)
	{
		line_follow.kp=fval;
		printf("Line kp=%.3f\r\n",fval);
		}
	else if(sscanf(cmd,"lki %f",&fval)==1)
	{
		line_follow.ki=fval;
		printf("Line ki=%.3f\r\n",fval);
		}
	else if(sscanf(cmd,"lkd %f",&fval)==1)
	{
		line_follow.kd=fval;
		printf("Line kd=%.3f\r\n",fval);
		}
		else if(strcmp(cmd,"lfdbg")==0)
	{
		lf_dbg_enable = !lf_dbg_enable;
		printf("LF debug %s\r\n", lf_dbg_enable ? "ON" : "OFF");
		}
else if(sscanf(cmd,"rroff %d",&ival)==1)
		{
			line_follow.rr_offset=(int16_t)ival;
			printf("RR offset=%d\r\n",ival);
			}
		/* ---- visual servo PID tuning ---- */
	else if(sscanf(cmd,"vkp %f",&fval)==1)
		{ visual_servo.kp_lat=fval; printf("VS lat kp=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vki %f",&fval)==1)
		{ visual_servo.ki_lat=fval; printf("VS lat ki=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vkd %f",&fval)==1)
		{ visual_servo.kd_lat=fval; VisualServo_Reset(&visual_servo);
		  printf("VS lat kd=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vkp2 %f",&fval)==1)
		{ visual_servo.kp_long=fval; printf("VS long kp=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vki2 %f",&fval)==1)
		{ visual_servo.ki_long=fval; printf("VS long ki=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vkd2 %f",&fval)==1)
		{ visual_servo.kd_long=fval; VisualServo_Reset(&visual_servo);
		  printf("VS long kd=%.3f\r\n",(double)fval); }
	/* ---- LLM PID Tuner 写回: config pid.x/pid.y (stm32f407_openmv) ---- */
	else if(sscanf(cmd,"config pid.x.kp %f",&fval)==1)
		{ visual_servo.kp_lat=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.x.kp=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"config pid.x.ki %f",&fval)==1)
		{ visual_servo.ki_lat=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.x.ki=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"config pid.x.kd %f",&fval)==1)
		{ visual_servo.kd_lat=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.x.kd=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"config pid.y.kp %f",&fval)==1)
		{ visual_servo.kp_long=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.y.kp=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"config pid.y.ki %f",&fval)==1)
		{ visual_servo.ki_long=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.y.ki=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"config pid.y.kd %f",&fval)==1)
		{ visual_servo.kd_long=fval; VisualServo_Reset(&visual_servo);
		  printf("pid.y.kd=%.3f\r\n",(double)fval); }
	else if(sscanf(cmd,"vdist %d",&ival)==1&&ival>0)
		{ visual_servo.target_distance_cm=(int16_t)ival; VisualServo_Reset(&visual_servo);
		  printf("VS dist=%d cm\r\n",ival); }
	else if(sscanf(cmd,"vcx %d",&ival)==1&&ival>=0&&ival<=320)
		{ visual_servo.target_cx=(int16_t)ival; VisualServo_Reset(&visual_servo);
		  printf("VS cx=%d px\r\n",ival); }
	else if(strcmp(cmd,"vpid")==0)
	{
		printf("--- Visual Servo PID ---\r\n");
		printf(" lat : kp=%.3f ki=%.3f kd=%.3f\r\n",
			(double)visual_servo.kp_lat,(double)visual_servo.ki_lat,(double)visual_servo.kd_lat);
		printf(" long: kp=%.3f ki=%.3f kd=%.3f\r\n",
			(double)visual_servo.kp_long,(double)visual_servo.ki_long,(double)visual_servo.kd_long);
		printf(" tgt : cx=%d dist=%d cm speed=%d\r\n",
			(int)visual_servo.target_cx,(int)visual_servo.target_distance_cm,(int)visual_servo.base_speed_max);
		printf(" aligned: %s lost=%d\r\n",
			VisualServo_IsAligned(&visual_servo)?"YES":"NO",(int)visual_servo.lost_cnt);
		}
	else if(sscanf(cmd,"mode %d",&ival)==1&&ival>=0&&ival<=3)
	{
		work_mode=(uint8_t)ival;
		printf("Work mode set to %d\r\n",work_mode);
		}
	else if(sscanf(cmd,"line_time %d",&ival)==1&&ival>0)
	{
		line_time_ticks=(int32_t)(g_sys_tick + (uint32_t)ival * 100);
		work_mode=MODE_LINE_FOLLOW;
		printf("Line follow %d s\r\n",ival);
		}
	else if(sscanf(cmd,"enable %d",&ival)==1&&(ival==0||ival==1))
	{
		motor_enable=(uint8_t)ival;
		printf("Motor enable=%d\r\n",ival);
		}
	else if(sscanf(cmd,"SETPOINT:%d",&spd)==1)
	{
		for(i=0;i<4;i++)target_speed[i]=spd;
		printf("Setpoint %d\r\n",spd);
		}
	else if(strcmp(cmd,"show")==0)
	{
		printf("=== Motor PID ===\r\n");
		for(i=0;i<4;i++)
			printf("M[%d] Kp=%.3f Ki=%.3f Kd=%.3f spd=%d pos=%ld\r\n",
				i,pid_motor[i].Kp,pid_motor[i].Ki,pid_motor[i].Kd,
				target_speed[i],(long)Encoder_GetPosition(i));
		}
	else if(sscanf(cmd,"servo %d %d",&servo_id,&angle)==2&&servo_id<5)
	{
		smoothservo_StopOne(servo_id);
		Servo_PWMEnable(servo_id,1);
		Servo_SetAngle(servo_id,angle);
		printf("Servo %d -> %d\r\n",servo_id,angle);
		}
	else if(sscanf(cmd,"s_smooth %d %d %d",&servo_id,&angle,&time_ms)==3)
	{
		smooth_Settarget(servo_id,angle,time_ms);
		printf("Servo %d smooth to %d in %d ms\r\n",servo_id,angle,time_ms);
		}
	else if(strcmp(cmd,"3")==0)
	{
		uint16_t scan_angle;
		int poll_i;
		uint8_t any_found;
		OpenMV_Data omv;

		printf("Trailer scan\r\n");
		smooth_Settarget(0, 170, 800);
		smooth_Settarget(1, 110, 800);
		smooth_Settarget(2, 100, 800);
		while (smoothservo_AnyBusy())
			Delay_ms(10);
		Delay_ms(300);

		OpenMV_FlushRx();
		OpenMV_SendCmd("MODE,APRILTAG");
		Delay_ms(500);

		any_found = 0;
		for (scan_angle = 150; scan_angle <= 200; scan_angle += 5) {
			smooth_Settarget(0, scan_angle, 600);
			while (smoothservo_IsBusy(0))
				Delay_ms(10);
			Delay_ms(300);
			for (poll_i = 0; poll_i < 5; poll_i++) {
				OpenMV_PollLine();
				if (OpenMV_GetData(&omv) && omv.cx > 0) {
					printf("  ID=%d cx=%d waist=%d\r\n",
					       (int)omv.tag_id, (int)omv.cx,
					       (int)scan_angle);
					any_found = 1;
				}
				Delay_ms(50);
			}
		}
		if (!any_found)
			printf("  no tags\r\n");
		printf("Trailer scan done\r\n");
	}
	else if(strcmp(cmd,"grasp")==0)
	{
		if(ArmSM_GetState()==ARM_IDLE)
		{
			ArmSM_RequestGrasp();
			Action_Grasp();
			ArmSM_NotifyComplete();
			}
		else printf("Arm busy\r\n");
		}
	else if(sscanf(cmd,"place %d",&angle)==1)
	{
		if(ArmSM_GetState()==ARM_IDLE)
		{
			ArmSM_RequestPlace(angle);
			Action_Place(angle);
			ArmSM_NotifyComplete();
			}
		else printf("Arm busy\r\n");
		}
	else if(sscanf(cmd,"a_set %d %d",&ival,&angle)==2)
		Action_SetAngle((uint8_t)ival,angle);
	else if(sscanf(cmd,"t_set %d %d",&ival,&angle)==2)
		Action_SetTime((uint8_t)ival,angle);
	else if(strcmp(cmd,"show_act")==0)
		Action_ShowParams();
	else if(strcmp(cmd,"servo_off")==0)
	{
		for(i=0;i<5;i++)Servo_PWMEnable(i,0);
		smoothservo_EmergencyStop();
		printf("All servos OFF\r\n");
		}
	else if(strcmp(cmd,"servo_on")==0)
	{
		for(i=0;i<5;i++){Servo_PWMEnable(i,1);Servo_SetAngle(i,90);}
		printf("All servos ON (90 deg)\r\n");
		}
	else if(strcmp(cmd,"reset_arm")==0)
	{
		if(ArmSM_GetState()!=ARM_GRASPING&&ArmSM_GetState()!=ARM_PLACING)
		{
			ArmSM_RequestReset();
			Action_Reset();
			ArmSM_NotifyComplete();
			}
		else printf("Arm busy\r\n");
		}
	else if(strcmp(cmd,"stop")==0)
	{
		for(i=0;i<4;i++)
		{
			target_speed[i]=0;
			Motor_SetSpeed(i,0);
			PID_Reset(&pid_motor[i]);
			}
		work_mode=MODE_MANUAL;
		printf("Emergency stop\r\n");
		}
	else if(strcmp(cmd,"BT")==0)
		printf("BT OK\r\n");
	/* ---- 视觉指令 ---- */
	else if(sscanf(cmd,"vmode %d",&ival)==1&&ival>=0&&ival<=3)
	{
		OpenMV_SetMode((uint8_t)ival);
		printf("Vision mode set to %d\r\n",ival);
		}
	else if(sscanf(cmd,"vcolor %d",&ival)==1&&ival>=1&&ival<=3)
	{
		OpenMV_SetTargetColor((uint8_t)ival);
		printf("Target color set to %d\r\n",ival);
		}
	else if(sscanf(cmd,"vtag %d",&ival)==1)
	{
		OpenMV_SetTargetTag((int16_t)ival);
		printf("Target tag set to %d\r\n",ival);
		}
	else if(strcmp(cmd,"vqr")==0)
	{
		OpenMV_SetMode(OMV_MODE_QRCODE);
		printf("QR scan mode enabled\r\n");
		}
	else if(strcmp(cmd,"vdata")==0)
	{
		OpenMV_Data d;
		if(OpenMV_GetData(&d))
			printf("Vision: tag=%d cx=%d cy=%d dist=%d cm angle=%d deg\r\n",
				(int)d.tag_id,(int)d.cx,(int)d.cy,
				(int)d.distance_cm,(int)d.angle_deg);
		else
			printf("Vision: no fresh data\r\n");
		}
	else if(strcmp(cmd,"vqrtext")==0)
	{
		OpenMV_QRData qr;
		if(OpenMV_GetQRData(&qr))
			printf("QR text: %s\r\n",qr.text);
		else
			printf("No QR data\r\n");
		}
	else if(strcmp(cmd,"vcls")==0)
	{
		OpenMV_CLSData cls;
		if(OpenMV_GetCLSData(&cls))
			printf("AI: class=%d confidence=%d%%\r\n",
				(int)cls.class_id,(int)cls.confidence);
		else
			printf("No AI classification data\r\n");
		}
	else if(strcmp(cmd,"vstat")==0)
	{
		OpenMV_PrintStatus();
		}
	else if(strcmp(cmd,"trycolor")==0)
	{
		/* 手动测试 COLOR 模式: 切过去→轮询→报告→切回AI */
		uint32_t t0;
		OpenMV_Data td;
		int ok;
		ok = 0;
		OpenMV_SendCmd("MODE,COLOR,1");
		/* 丢旧数据 */
		OpenMV_PollLine();
		{
			OpenMV_Data d;
			(void)OpenMV_GetData(&d);
			}
		t0 = g_sys_tick;
		printf("trycolor: waiting for COLOR $TAG...\r\n");
		while (g_sys_tick - t0 < 200) {
			OpenMV_PollLine();
			if (OpenMV_IsFresh()) {
				(void)OpenMV_GetData(&td);
				ok = 1;
				break;
				}
			}
		if (ok) {
			printf("trycolor: OK! $TAG after %lu ticks\r\n",
			       (unsigned long)(g_sys_tick - t0));
			printf("  tag=%d cx=%d cy=%d dist=%d cm\r\n",
			       (int)td.tag_id, (int)td.cx, (int)td.cy,
			       (int)td.distance_cm);
			} else {
			printf("trycolor: FAIL no $TAG after 2s\r\n");
			}
		/* 切回 AI 不破坏管线 */
		OpenMV_SendCmd("MODE,AI");
		}
	else if(strcmp(cmd,"qrdata")==0)
	{
		OpenMV_QRData qr;
		OpenMV_Data d;
		if(OpenMV_GetQRData(&qr))
			printf("QR text: %s\r\n", qr.text);
		else
			printf("QR text: (none)\r\n");
		if(OpenMV_GetData(&d))
			printf("QR pos: tag=%d cx=%d cy=%d dist=%d cm w=%d\r\n",
				(int)d.tag_id, (int)d.cx, (int)d.cy,
				(int)d.distance_cm, (int)d.pixel_width);
		else
			printf("QR pos: (none)\r\n");
		}
	else if(strcmp(cmd,"qrtrig")==0)
	{
		qr_trig_flag = 1;
		printf("QR trigger set, will enter AI scan\r\n");
		}
		else if(strcmp(cmd,"ai_stat")==0)
		{
			ai_dbg_stat = 1;
			printf("AI stat requested\r\n");
		}
		else if(strcmp(cmd,"ai_skip")==0)
		{
			ai_dbg_skip = 1;
			printf("AI skip requested\r\n");
			}
		else if(sscanf(cmd,"ai_go %d",&ival)==1)
		{
			ai_dbg_phase = (uint8_t)ival;
			ai_dbg_go = 1;
			printf("AI go to phase %d\r\n", (int)ai_dbg_phase);
			}
else if(strcmp(cmd,"restart")==0)
	{
		extern volatile uint8_t ai_restart_flag;
		ai_restart_flag = 1;
		printf("AI pipeline restart requested\r\n");
	}
else if(strcmp(cmd,"run")==0)
	{
		/* run: one-key start/restart whole flow (AI pipeline + dock + stations) */
		extern volatile uint8_t ai_restart_flag;
		ai_restart_flag = 1;
		printf("RUN: whole flow restart (crossroad dock -> Station0 -> Station1)\r\n");
	}
else if(strcmp(cmd,"flow")==0)
	{
		extern volatile uint8_t flow_dbg;
		flow_dbg = 1;
		printf("FLOW dump requested\r\n");
	}
else if(strcmp(cmd,"ls")==0)
	{
		bool st[5];
		LineSensor_Read(st);
		printf("LS deb: [%d %d %d %d %d] (R1 R2 C L2 L1) all_on=%d\r\n",
			st[0], st[1], st[2], st[3], st[4], (int)LineSensor_AllOn());
		printf("LS raw: [%d %d %d %d %d]\r\n",
			(int)(GPIO_ReadInputDataBit(SENSOR_PORT_RIGHT1, SENSOR_PIN_RIGHT1) != 0),
			(int)(GPIO_ReadInputDataBit(SENSOR_PORT_RIGHT2, SENSOR_PIN_RIGHT2) != 0),
			(int)(GPIO_ReadInputDataBit(SENSOR_PORT_CENTER, SENSOR_PIN_CENTER) != 0),
			(int)(GPIO_ReadInputDataBit(SENSOR_PORT_LEFT2, SENSOR_PIN_LEFT2) != 0),
			(int)(GPIO_ReadInputDataBit(SENSOR_PORT_LEFT1, SENSOR_PIN_LEFT1) != 0));
	}
else if(sscanf(cmd,"dock_force %d",&ival)==1 && ival>=0 && ival<=6)
	{
		extern volatile uint8_t dock_state;
		extern volatile uint32_t dock_tick;
		extern volatile uint8_t dock_lost;
		extern volatile uint8_t dock_offline_cnt;
		dock_state = (uint8_t)ival;
		dock_tick = g_sys_tick;
		dock_lost = 0;
		dock_offline_cnt = 0;
		printf("DOCK: force state=%d (tick reset)\r\n", ival);
	}
else if(strcmp(cmd,"arm")==0)
	{
		int ai;
		printf("ArmSM: %s busy=%d\r\n",
			ArmSM_StateName(ArmSM_GetState()), (int)ArmSM_IsBusy());
		for (ai = 0; ai < 5; ai++)
			printf("  srv%d: cur=%.0f busy=%d\r\n",
				ai, (double)smooth_GetCurrentAngle(ai),
				(int)smoothservo_IsBusy(ai));
	}
else if(strcmp(cmd,"pid_all")==0)
	{
		int pi;
		printf("--- Motor speed PID ---\r\n");
		for (pi = 0; pi < 4; pi++)
			printf("M%d spd Kp=%.3f Ki=%.3f Kd=%.3f | pos Kp=%.3f Ki=%.3f Kd=%.3f\r\n",
				pi, (double)pid_motor[pi].Kp, (double)pid_motor[pi].Ki,
				(double)pid_motor[pi].Kd,
				(double)pid_positon[pi].Kp, (double)pid_positon[pi].Ki,
				(double)pid_positon[pi].Kd);
		printf("Line: base=%d Kp=%.3f Ki=%.3f Kd=%.3f rr=%d curve=%d/%d\r\n",
			(int)line_follow.base_speed, (double)line_follow.kp,
			(double)line_follow.ki, (double)line_follow.kd,
			(int)line_follow.rr_offset, (int)line_follow.curve_count,
			(int)line_follow.curve_target);
	}
else if(strcmp(cmd,"enc_all")==0)
	{
		int ei;
		for (ei = 0; ei < 4; ei++)
			printf("Enc%d: pos=%ld spd=%d max=%d dir=%d\r\n",
				ei, (long)Encoder_GetPosition(ei), Encoder_getspeed(ei),
				encoder_max[ei], (int)encoder_dir[ei]);
	}
else if(strcmp(cmd,"uptime")==0)
	{
		printf("Uptime: %lu ticks = %lu.%02lus\r\n",
			(unsigned long)g_sys_tick,
			(unsigned long)(g_sys_tick / 100),
			(unsigned long)(g_sys_tick % 100));
	}
else if(strcmp(cmd,"status")==0)
	{
		/* LLM PID Tuner: 立即输出一整套 Status 快照 */
		sendVisualServoTelemetry();
	}
else if(strcmp(cmd,"dock_test")==0)
	{
		extern volatile uint8_t dock_test_flag;
		dock_test_flag = 1;
		printf("Dock test: reverse dock now\r\n");
	}
else if(strcmp(cmd,"dock_tune")==0)
	{
		extern volatile uint8_t dock_tune_flag;
		extern volatile uint8_t trace_enable;
		dock_tune_flag = 1;
		trace_enable = 1;   /* 一键启动: 自动开 trace, 不用单独 trace 1 */
		printf("Dock tune: auto-loop reverse dock (trace ON)\r\n");
	}
else if(strcmp(cmd,"telem")==0)
	{
		extern volatile uint8_t telem_enable;
extern volatile uint8_t lf_dbg_enable;
		telem_enable = !telem_enable;
		printf("Telemetry %s\r\n", telem_enable ? "ON" : "OFF");
	}
else if(sscanf(cmd,"telem %d",&ival)==1)
	{
		extern volatile uint8_t telem_enable;
extern volatile uint8_t lf_dbg_enable;
		telem_enable = (uint8_t)(ival ? 1 : 0);
		printf("Telemetry %s\r\n", telem_enable ? "ON" : "OFF");
	}
else if(strcmp(cmd,"dockstat")==0)
	{
		extern volatile uint8_t dock_state;
		extern volatile uint32_t dock_tick;
		extern volatile uint8_t dock_lost;
		extern volatile uint8_t dock_offline_cnt;
		extern volatile uint8_t dock_tune_on;
		extern volatile uint8_t dock_test_flag;
		extern volatile uint8_t dock_tune_flag;
		extern volatile uint8_t trailer_dock_flag;
		extern volatile uint8_t trailer_docked;
		printf("DOCK: state=%d lost=%u offline=%u tick=%lu\\r\\n",
			(unsigned int)dock_lost, (unsigned int)dock_offline_cnt,
			(unsigned long)dock_tick);
		printf("DOCK: tune_on=%d test_f=%d tune_f=%d cross_f=%d docked=%d\\r\\n",
			(int)dock_tune_on, (int)dock_test_flag,
			(int)dock_tune_flag, (int)trailer_dock_flag,
			(int)trailer_docked);
	}
else if(strcmp(cmd,"crossdock")==0)
	{
		extern volatile uint8_t cross_dock_enable;
		cross_dock_enable = !cross_dock_enable;
		printf("Cross-dock %s\r\n", cross_dock_enable ? "ENABLED" : "DISABLED");
	}
else if(sscanf(cmd,"crossdock %d",&ival)==1)
	{
		extern volatile uint8_t cross_dock_enable;
		cross_dock_enable = (uint8_t)(ival ? 1 : 0);
		printf("Cross-dock %s\r\n", cross_dock_enable ? "ENABLED" : "DISABLED");
	}
else if(strcmp(cmd,"vsdbg")==0)
	{
		extern VisualServo_Handle visual_servo;
		extern volatile uint8_t motor_enable;
		extern volatile uint8_t work_mode;
		extern volatile uint8_t dock_state;
		extern volatile uint8_t dock_lost;
		extern volatile uint8_t dock_tune_on;
		OpenMV_Data d;
		int have;
		printf("VS lat: kp=%.3f ki=%.3f kd=%.3f\\r\\n",
			(double)visual_servo.kp_lat, (double)visual_servo.ki_lat,
			(double)visual_servo.kd_lat);
		printf("VS long: kp=%.3f ki=%.3f kd=%.3f\\r\\n",
			(double)visual_servo.kp_long, (double)visual_servo.ki_long,
			(double)visual_servo.kd_long);
		printf("VS tgt: cx=%d dist=%d base=%d turnlim=%d lostthr=%d\\r\\n",
			(int)visual_servo.target_cx, (int)visual_servo.target_distance_cm,
			(int)visual_servo.base_speed_max, (int)visual_servo.turn_limit,
			(int)visual_servo.lost_threshold);
		printf("VS st: en=%d wm=%d dock_st=%d dock_lost=%d tune=%d\\r\\n",
			(int)motor_enable, (int)work_mode,
			(int)dock_state, (int)dock_lost, (int)dock_tune_on);
		printf("VS spd: %d,%d,%d,%d\\r\\n",
			(int)target_speed[0], (int)target_speed[1],
			(int)target_speed[2], (int)target_speed[3]);
		have = OpenMV_GetData(&d);
		if (have)
			printf("VS omv: tag=%d cx=%d dist=%d err_lat=%d\\r\\n",
				(int)d.tag_id, (int)d.cx, (int)d.distance_cm,
				(int)(visual_servo.target_cx - d.cx));
		else
			printf("VS omv: no fresh data\\r\\n");
	}
else if(strcmp(cmd,"trace")==0)
	{
		extern volatile uint8_t trace_enable;
		trace_enable = !trace_enable;
		printf("Trace %s\\r\\n", trace_enable ? "ON" : "OFF");
	}
else if(sscanf(cmd,"trace %d",&ival)==1)
	{
		extern volatile uint8_t trace_enable;
		trace_enable = (uint8_t)(ival ? 1 : 0);
		printf("Trace %s\\r\\n", trace_enable ? "ON" : "OFF");
	}
else if(strcmp(cmd,"0")==0)
		{
			smooth_Settarget(0, 60, 800);
			smooth_Settarget(1, 50, 800);
			smooth_Settarget(2, 160, 800);
			smooth_Settarget(3, 50, 500);
			printf("Observe: 0=60 1=50 2=160 3=50\r\n");
			}
	else if(strcmp(cmd,"1")==0)
		{
			if(ArmSM_GetState()==ARM_IDLE)
			{
				ArmSM_RequestGrasp();
				Action_Grasp();
				ArmSM_NotifyComplete();
				}
			else printf("Arm busy\r\n");
			}
	else if(strcmp(cmd,"2")==0)
		{
			Action_Reset();
			printf("Reset done\r\n");
			}
	else if(strcmp(cmd,"delivery")==0)
	{
		extern uint8_t g_delivery_mode;
		g_delivery_mode = 1;
		printf("Delivery mode ON: next station = delivery\r\n");
	}
	else if(strcmp(cmd,"help")==0)
	{
		printf("Commands:\r\n");
		printf("kp/ki/kd <id> <val>   spd <id> <val>\r\n");
		printf("m f|b|l|r|s   mode <0|1|2>\r\n");
		printf("enc_pos/speed/max/dir/clear <id>\r\n");
		printf("servo <id> <angle>   s_smooth <id> <angle> <ms>\r\n");
		printf("grasp   place <waist>   reset_arm\r\n");
		printf("a_set <id> <val>   t_set <id> <val>   show_act\r\n");
		printf("servo_off   servo_on   stop   show   help\r\n");
		printf("line_time <sec>   task_start <a> <b>\r\n");
		printf("vmode <0|1|2|3>   vcolor <1|2|3>   vtag <id>\r\n");
		printf("vfind <1..3>   vqrgo   vfind_ai   vstop\r\n");
		printf("vdata   vqrtext   vqr   vcls   vstat   trycolor\r\n");
		printf("qrdata   qrtrig\r\n");
			printf("ai_stat   ai_skip   ai_go <phase>\r\n");
			printf("vkp/vki/vkd <val>   vkp2/vki2/vkd2 <val>\r\n");
			printf("vdist <cm>   vcx <px>   vpid\r\n");
			printf("run   restart whole flow\r\n");
			printf("crossdock <0|1>   dockstat   dock_test   dock_tune\r\n");
			printf("flow   ls   dock_force <0-6>   arm   pid_all   enc_all   uptime\r\n");
		}
	else if(strcmp(cmd,"reset")==0)
	{
		for(i=0;i<4;i++)target_speed[i]=0;
		printf("System reset\r\n");
		}
	else if(strcmp(cmd,"save")==0)
	{
		Save_parameters();
		printf("Parameters saved (placeholder)\r\n");
		}
	else if(strcmp(cmd,"load")==0)
	{
		Load_parameters();
		printf("Parameters loaded (placeholder)\r\n");
		}
}
