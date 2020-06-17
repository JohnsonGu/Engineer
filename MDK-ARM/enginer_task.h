#ifndef ENGINER_TASK_H
#define ENGINER_TASK_H

#include "main.h"
#include "pid.h"


#define INIT_CURRENT       1800

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

#define MOTOR_ECD_TO_ANGLE          0.000039952259f

#define M3508_MOTOR_POSITION_PID_KP 1500.0f
#define M3508_MOTOR_POSITION_PID_KI 0.0f
#define M3508_MOTOR_POSITION_PID_KD 0.0f
#define M3508_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M3508_MOTOR_POSITION_PID_MAX_IOUT 400.0f

#define M3508_MOTOR_SPEED_PID_KP 10.0f
#define M3508_MOTOR_SPEED_PID_KI 1.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

typedef struct
{
    motor_measure_t *enginer_motor_measure;
		int16_t  target_speed;//目标速度
		float target_angle;//目标角度
		float angle;//累积角度
		int16_t give_current;
} enginer_motor_t;


typedef struct
{
    const RC_ctrl_t *enginer_rc_ctrl;
    enginer_motor_t enginer_yaw_motor;
	  enginer_motor_t enginer_pitch_motor;
	  pid_type_def motor_speed_pid[2];             //motor speed PID.底盘电机速度pid
		pid_type_def motor_angle_pid[2];              //follow angle PID.底盘跟随角度pid
} enginer_control_t;

typedef enum
{
	init,
	pick,
	put,
	in,
	out
} enginer_state;

extern void enginer_task(void const *pvParameters);

#endif
