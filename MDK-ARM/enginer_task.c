#include "enginer_task.h"
#include "main.h"
#include "cmsis_os.h"

enginer_control_t enginer_move;

enginer_state nowState = init;
int timeCount = 0;
int16_t angleCount = 0;
static void enginer_init(enginer_control_t *enginer_move_init)
{
	const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
	const static fp32 motor_positon_pid[3] = {M3508_MOTOR_POSITION_PID_KP, M3508_MOTOR_POSITION_PID_KI, M3508_MOTOR_POSITION_PID_KD};
    //get remote control point
    //��ȡң����ָ��
    enginer_move_init->enginer_rc_ctrl = get_remote_control_point();
	    //get chassis motor data point,  initialize motor speed PID
    //��ȡ���̵������ָ�룬��ʼ��PID 
	    enginer_move_init->enginer_yaw_motor.enginer_motor_measure = get_yaw_gimbal_motor_measure_point();
			enginer_move_init->enginer_pitch_motor.enginer_motor_measure = get_pitch_gimbal_motor_measure_point();
		for(int i=0;i<2;i++)
	{
		PID_init(&enginer_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init(&enginer_move_init->motor_angle_pid[i], PID_POSITION, motor_positon_pid, M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT);
	}
}

static void enginer_update(void)
{
				if(nowState == init)
			{
				enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd_count =0 ;
				enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd_count =0 ;
			}
		enginer_move.enginer_pitch_motor.angle = (enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
		enginer_move.enginer_yaw_motor.angle = (enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;	
}

void PIDrealize(void)
{
			if(nowState == init)
			{
			enginer_move.enginer_pitch_motor.give_current = INIT_CURRENT;
			enginer_move.enginer_yaw_motor.give_current = -INIT_CURRENT;//������෴
			}
			else
			{
			enginer_move.enginer_pitch_motor.target_speed = PID_calc(&enginer_move.motor_angle_pid[0],enginer_move.enginer_pitch_motor.angle, enginer_move.enginer_pitch_motor.target_angle);
			enginer_move.enginer_yaw_motor.target_speed = PID_calc(&enginer_move.motor_angle_pid[1],enginer_move.enginer_yaw_motor.angle,enginer_move.enginer_yaw_motor.target_angle);
			enginer_move.enginer_pitch_motor.give_current = PID_calc(&enginer_move.motor_speed_pid[0],enginer_move.enginer_pitch_motor.enginer_motor_measure->speed_rpm, enginer_move.enginer_pitch_motor.target_speed);
			enginer_move.enginer_yaw_motor.give_current = PID_calc(&enginer_move.motor_speed_pid[1],enginer_move.enginer_yaw_motor.enginer_motor_measure->speed_rpm, enginer_move.enginer_yaw_motor.target_speed);
			}
}


void enginer_task(void const *pvParameters)
{
		osDelay(100);
		enginer_init(&enginer_move);
	while(1)
	{
		enginer_update();
		
		switch(nowState)
		{
			case init://�����������ڳ�ʼ��֮���ٶȶ�С�ڶ�ֵ����ô�͵���ԭ��
				timeCount++;
				if(timeCount >2000)		
				{			
					nowState = in;
					timeCount = 0;
				}
				//if(enginer_move.enginer_pitch_motor.enginer_motor_measure->speed_rpm < 2 && enginer_move.enginer_yaw_motor.enginer_motor_measure->speed_rpm < 2)
				//nowState = in;
				break;
			case in://�����棬��צ��������

				if(switch_is_mid(enginer_move.enginer_rc_ctrl->rc.s[1]))//ң�������м�ʱ����צ������ת
				{
					nowState = put;
					timeCount = 0;
				}
					
				else
				{
					enginer_move.enginer_pitch_motor.target_angle = -0.6;
					enginer_move.enginer_yaw_motor.target_angle = 0.6;
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay3_Pin, GPIO_PIN_RESET);
				}
				break;
			case out://�������״̬
				if(switch_is_down(enginer_move.enginer_rc_ctrl->rc.s[1]))
				{
					nowState = pick;
					timeCount = 0;
				}
				else
				{
				enginer_move.enginer_pitch_motor.target_angle = -3;
				enginer_move.enginer_yaw_motor.target_angle = 3;
				}
				break;
			case put://����Ѿ���ת�����棬��ô״̬��Ϊ���棬��צ�ڰ�·�ɿ�(���ڳ�ȥ��״̬)
				if(enginer_move.enginer_pitch_motor.angle <= -2.5)
				{
					nowState = out;
					timeCount = 0;
				}

				if(enginer_move.enginer_pitch_motor.angle < -1)
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
				
				timeCount++;
				HAL_GPIO_WritePin(Relay_GPIO_Port, Relay2_Pin, GPIO_PIN_SET);
				if(timeCount  > 1000)
				{
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay3_Pin, GPIO_PIN_SET);
				}
					if(timeCount  > 2000)
				{
					enginer_move.enginer_pitch_motor.target_angle = -3;
					enginer_move.enginer_yaw_motor.target_angle = 3;
				}	

				break;
			case pick://��צ��������������ת����800���룬�ü�צ��ס��ҩ��

				HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
				timeCount++;

				if(timeCount >800)		
				{					
					
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay3_Pin, GPIO_PIN_RESET);
				}
				if(timeCount >1500)		
				{
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay2_Pin, GPIO_PIN_RESET);
				}
				if(timeCount >2000)
				{	
					enginer_move.enginer_pitch_motor.target_angle = 0;
					enginer_move.enginer_yaw_motor.target_angle = 0;
					
				}
			if(enginer_move.enginer_pitch_motor.angle > -1)
			{
				nowState = in;
				timeCount = 0;
			}
				break;
		}
		
		PIDrealize();
		CAN_cmd_gimbal(enginer_move.enginer_yaw_motor.give_current,enginer_move.enginer_pitch_motor.give_current,0,0);
		osDelay(1);
	}
}
