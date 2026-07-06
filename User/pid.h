#ifndef __pid_h
#define __pid_h

typedef enum {
	PID_POSITION,
	PID_INCREMENTAL
} PID_Type;

typedef struct {
	PID_Type type;
	float Kp, Ki, Kd;
	float Ts;
	float integral_limit;
	float out_limit;
	float integral_sep_th;

	float integral;
	float last_err;
	float last_feedback;

	float last_out;
	float last2_err;
	float last_deriv;

	uint8_t initialized;
} PID_Handle;

void PID_Reset(PID_Handle *pid);
void PID_Init(PID_Handle *pid, PID_Type type,
              float Kp, float Ki, float Kd,
              float integral_limit, float out_limit, float integral_sep_th);
float PID_Update(PID_Handle *pid, float setpoint, float feedback, float dt);
#endif
