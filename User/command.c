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
#include "smooth_servo.h"
#include "state_machine.h"
#include "action_group.h"
#include "OLED.h"
#include "command.h"
#include "Timer.h"
#include "commend_openmv.h"
#include "visual_servo.h"
#include "vision_task.h"
 extern VisualServo_Handle visual_servo;
 extern TaskQueue task_queue;
 extern volatile uint8_t qr_trig_flag;
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
extern int16_t encoder_max[4];
extern int8_t encoder_dir[4];
extern volatile int32_t line_time_ticks;
extern volatile uint32_t g_sys_tick;
extern VisionTask_Handle vision_task;

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
	else if(sscanf(cmd,"task_start %d %d",&ival,&spd)==2)
		TaskNav_Start(ival,spd);
	else if(strcmp(cmd,"task_stop")==0)
		TaskNav_Stop();
	else if(strcmp(cmd,"task_status")==0)
		printf("Task state=%d target=%ld\r\n",
			(int)TaskNav_GetState(),(long)TaskNav_GetTarget());
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
	else if(sscanf(cmd,"vfind %d",&ival)==1&&ival>=1&&ival<=3)
		VisionTask_StartColorSearch(&vision_task,(uint8_t)ival);
	else if(strcmp(cmd,"vqrgo")==0)
		VisionTask_StartQRSearch(&vision_task);
	else if(strcmp(cmd,"vfind_ai")==0)
		VisionTask_StartAIScan(&vision_task);
	else if(strcmp(cmd,"vstop")==0)
		VisionTask_Stop(&vision_task);
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
			printf("vkp/vki/vkd <val>   vkp2/vki2/vkd2 <val>\r\n");
			printf("vdist <cm>   vcx <px>   vpid\r\n");
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
