/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe  
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description: 
 * @Note:       1.��̨�����ʼ�� ���Կ��ǲ�ʹ�ô���pid�������Լ����� ���쵽��Ŀ��ֵ
				2.Ŀǰ�Ǽ�����̨��ʼ���Ƕ���Ϊ���Ƶĳ�ʼ�Ƕȣ�����mpu��λ��һ����
 * @Others: 
**/
#include "gimbal_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "pid.h"
#include "INS_task.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "monitor_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "start_task.h"
#include "gimbal_task.h"
#include "can2_app.h"
#include "flash.h"
#include "usart.h"
#include "stdlib.h"

#include "arm_math.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "shoot_task.h"
#include "SortAver_Filter.h"

/*���׿�����*/
#define KF_ANGLE 0
#define KF_SPEED 1
#define KF_ACCEL 2

extern _tx2_control_data control_data;
extern TaskHandle_t INS_Task_Handler;
extern monitor_t monitor;

gimbal_pid_t gimbal_pid;
gimbal_control_data_t gimbal_control_data;
robot_work_mode_e robot_work_mode;
robot_control_mode_e robot_control_mode;
gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;
speed_calc_data_t Vision_Yaw_speed_Struct; //Ԥ���ٶȽṹ��
speed_calc_data_t Vision_Pitch_speed_Struct;
kalman_filter_t yaw_kalman_filter; //�������ṹ��
kalman_filter_t pitch_kalman_filter;

kalman_filter_init_t yaw_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.002 /*0.001*/, 0, 1}, //����ʱ����
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {200, 0, 0, 400} //500 1000
};							   //��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.002 /*0.001*/, 0, 1}, //����ʱ����
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {200, 0, 0, 400}};						//��ʼ��pitch�Ĳ���kalman����
float constrain_float(float amt, float low, float high) //���ƺ���
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float(float final, float now, float ramp)
{
	float buffer = 0;
	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
		{
			now += ramp;
		}
		else
		{
			now += buffer;
		}
	}
	else
	{
		if (buffer < -ramp)
		{
			now += -ramp;
		}
		else
		{
			now += buffer;
		}
	}
	return now;
}

uint16_t Gimbal_init_direction_time = 0;
uint8_t Gimbal_init_flag = 1;

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//robot_control_mode
void set_robot_control_mode(robot_control_mode_e mode)
{
	robot_control_mode = mode;
}
uint8_t get_robot_control_mode(void)
{
	return robot_control_mode;
}
void robot_control_mode_update(RC_ctrl_t *rc_s) ///ң�������ƻ�����ģʽ
{
	switch (rc_s->rc.s2)
	{
	case RC_SW_UP:
		set_robot_control_mode(KEY_MOUSE_MODE);
		break;
	case RC_SW_MID:
		set_robot_control_mode(REMOTE_MODE);
		break;
	case RC_SW_DOWN:
		set_robot_control_mode(GUI_CALI_MODE);
		break;
	}
	// if(monitor.exist_error_flag == 1)//�������ش���ʱ������ң��ָ��ǿ��תΪ����ģʽ�����������ע�͵���һ���е�����߾��޷���
	// {
	// 	set_robot_control_mode(GUI_CALI_MODE);
	// }
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
uint8_t gimbal_position_init_finish_flag = 0;
//robot_work_mode
void set_robot_work_mode(robot_work_mode_e mode)
{
	robot_work_mode = mode;
}
uint8_t get_robot_work_mode(void)
{
	return robot_work_mode;
}
void robot_work_mode_update(RC_ctrl_t *rc_s)
{
	//work mode
	if (gimbal_position_init_finish_flag == 0 && get_robot_control_mode() != GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_INIT_MODE); //��ʼ��
		gimbal_position_init_finish_flag = 1;
	}
	else if (get_robot_control_mode() == REMOTE_MODE)
	{
		Gimbal_init_direction_time = 0;
		switch (rc_s->rc.s1)
		{
		case RC_SW_UP:
			set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
			break;
		case RC_SW_MID:
			set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);
			break;
		case RC_SW_DOWN:
			set_robot_work_mode(ROBOT_COMMON_MODE);
			break;
		default:
			set_robot_work_mode(ROBOT_COMMON_MODE);
		}
	}
	else if (get_robot_control_mode() == KEY_MOUSE_MODE)
	{
		Gimbal_init_direction_time = 0;
		if (rc_s->key.v & ROBOT_ROTATE_STOP_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
		}
		else if (rc_s->key.v & ROBOT_ROTATE_MOTION_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);
		}
		else if (rc_s->key.v & ROBOT_COMMON_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_COMMON_MODE);
			shoot_control_data.magazine_control_flag = 0;
			shoot_control_data.shoot_vision_flag = 0;
		}
	}
	else if (get_robot_control_mode() == GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_CALI_MODE);
		gimbal_position_init_finish_flag = 0;
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//gimbal_work_mode
void set_gimbal_work_mode(gimbal_work_mode_e mode)
{
	gimbal_work_mode = mode;
}
uint8_t get_gimbal_work_mode(void)
{
	return gimbal_work_mode;
}

void gimbal_work_mode_update(RC_ctrl_t *rc_s, gimbal_control_data_t *gimbal_control_data)
{
	robot_control_mode_update(rc_s);
	robot_work_mode_update(rc_s);

	if (get_robot_work_mode() == ROBOT_CALI_MODE)
	{
		set_gimbal_work_mode(GIMBAL_CALI_MODE);
	}
	else if (get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		set_gimbal_work_mode(GIMBAL_RELATIVE_ANGLE_MODE);
	}
	else
	{
		set_gimbal_work_mode(GIMBAL_ABSOLUTE_ANGLE_MODE);
	}

	switch (get_gimbal_work_mode())
	{
	case GIMBAL_RELATIVE_ANGLE_MODE: //��ԽǶȣ�ʹ�ñ���ֵ
	{
		gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
		gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
	}
	break;
	case GIMBAL_ABSOLUTE_ANGLE_MODE: //���ԽǶȣ�ʹ��������
	{
		gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
		gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
	}
	break;
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_val_limit(int16_t *val, int16_t min, int16_t max)
{
	if (*val < min)
	{
		*val = min;
	}
	else if (*val > max)
	{
		*val = max;
	}
}
void gimbal_val_limit_float(float *val, float min, float max)
{
	if (*val < min)
	{
		*val = min;
	}
	else if (*val > max)
	{
		*val = max;
	}
}
void mouse_sensit_cali(RC_ctrl_t *rc_ctrl)
{
	if (rc_ctrl->mouse.x < 20)
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT;
	}

	if (rc_ctrl->mouse.y < 20)
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT;
	}
}
float speed_threshold = 5.f; //�ٶȹ���
float debug_speed;			 //�����Ҹ�,һ�㶼��1����,debug��

float pitch_vision_Kp = 0.5;//0.5
float yaw_vision_Kp = 1;

float pitch_vision_Kd = 0.5;
float yaw_vision_Kd = 0.5;
/**
  * @brief      �Ӿ�ͬλ�ò���
  * @author         
  * @param[in]      
  * @retval			
  * @note        ������Ҫ��ȥ����ǰ���ƫ��Ƕ�
  */
float yaw_install_k = 0;
float pitch_install_k = 0;
void vision_install_compensate(void)
{
	control_data.yaw_compensate = (atan(yaw_install_Difference / control_data.Target_distance) * yaw_install_k);
	// if( control_data.pitch_dev > 0 )	//��Ҫ����̧ͷ��valueҪ���
	// {
	// 	control_data.pitch_compensate = (atan(pitch_install_Difference / control_data.Target_distance) *pitch_install_k );
	// }
	// else if ( control_data.pitch_dev > 0 )
	// {
	// 	control_data.pitch_compensate = - (atan(pitch_install_Difference / control_data.Target_distance) *pitch_install_k );
	// }
	control_data.pitch_compensate = -(atan(pitch_install_Difference / control_data.Target_distance) * pitch_install_k);
}
/**
  * @brief      �Ӿ�����
  * @author         
  * @param[in]      
  * @retval			
  * @note        yawͨ�������ǵı仯Ԥ��λ��  pitchͨ������任�߶�
  */

float yaw_INS_com_k =0; //0.40; ���ӱȽϺ�
float pitch_distance_com_k = 1.2;//3.5;
#define pitch_distance_compensation_constant 700
void vision_INS_distance_compensation(_tx2_control_data* control_data,gimbal_control_data_t* gimbal_control_data)
{  
	float yaw_INS;
	static float yaw_INS_old=0,yaw_INS_err=0;

	yaw_INS=gimbal_control_data->gimbal_INS->yaw_angle;
	yaw_INS_err=yaw_INS-yaw_INS_old;
	if(yaw_INS_err>5||yaw_INS_err<-5)
		yaw_INS_err=0;
	yaw_INS_err=constrain_float(yaw_INS_err,-2,2);
	if(((yaw_INS_err)>1.0f)||((yaw_INS_err)<-1.0f))//��С�ı仯��Ҫ
		control_data->yaw_INS_compensation=-(yaw_INS_err)*yaw_INS_com_k;
	else
	control_data->yaw_INS_compensation=0;

	yaw_INS_old=yaw_INS;
	
	
//	if(control_data->Target_distance<6000)
//	{
			control_data->pitch_distance_compensation=(-pitch_distance_com_k)*
												log(control_data->Target_distance/pitch_distance_compensation_constant+1);///log(2.8);
	
	//}//log(8)/log(2)  log2(8)
	
}
/**
  * @brief      ��������
  * @author         
  * @param[in]      
  * @retval			
  * @note        �����ֵ�����ӣ����ɸ�      *hyj
  */
float vision_G = 54;          //����̫��
uint16_t Gravity_distance;
float GG;
char count=0;
extern ext_Judge_data_t Judge_data;
void vision_Gravity_compensate(void)
{ 
	//uint16_t Gravity_distance;
//	Gravity_distance = control_data.Target_distance/1000; 
//  value_buf[count] = control_data.Target_distance;
//	count=count+1;
//	if(count==N-1)
//	{
//		count=0;
		Gravity_distance = control_data.Target_distance/100;
     //�����
//		switch (Gravity_distance)
//		{
//			case 10:control_data.Gravity_compensation = 0.15;break;
//			case 15:control_data.Gravity_compensation = 0.001;break;
//			case 16:control_data.Gravity_compensation = 0.03;break;
//			case 17:control_data.Gravity_compensation = 0.03;break;
//			case 18:control_data.Gravity_compensation = 0.03;break;
//			case 19:control_data.Gravity_compensation = 0.03;break;
//			case 20:control_data.Gravity_compensation = 0.03;break;
//			case 21:control_data.Gravity_compensation = 0.03;break;
//			case 22:control_data.Gravity_compensation = 0.03;break;
//			case 23:control_data.Gravity_compensation = 0.03;break;
//			case 24:control_data.Gravity_compensation = 0.03;break;
//			case 25:control_data.Gravity_compensation = 0.053;break;
//			case 26:control_data.Gravity_compensation = 0.053;break;
//			case 27:control_data.Gravity_compensation = 0.053;break;
//			case 28:control_data.Gravity_compensation = 0.053;break;
//			case 29:control_data.Gravity_compensation = 0.053;break;
//			case 30:control_data.Gravity_compensation = 0.053;break;
//			case 31:control_data.Gravity_compensation = 0.053;break;
//			case 32:control_data.Gravity_compensation = 0.06;break;
//			case 33:control_data.Gravity_compensation = 0.06;break;
//			case 34:control_data.Gravity_compensation = 0.06;break;
//			case 35:control_data.Gravity_compensation = 0.06;break;
//			case 36:control_data.Gravity_compensation = 0.075;break;
//			case 37:control_data.Gravity_compensation = 0.075;break;
//			case 38:control_data.Gravity_compensation = 0.075;break;
//			case 39:control_data.Gravity_compensation = 0.075;break;
//			case 40:control_data.Gravity_compensation = 0.075;break;
//			case 41:control_data.Gravity_compensation = 0.075;break;
//			case 42:control_data.Gravity_compensation = 0.09;break;
//			case 43:control_data.Gravity_compensation = 0.09;break;
//			case 44:control_data.Gravity_compensation = 0.09;break;
//			case 45:control_data.Gravity_compensation = 0.09;break;
//			case 46:control_data.Gravity_compensation = 0.09;break;
//			case 47:control_data.Gravity_compensation = 0.09;break;
//			case 48:control_data.Gravity_compensation = 0.09;break;
//			case 49:control_data.Gravity_compensation = 0.09;break;
//			case 50:control_data.Gravity_compensation = 0.1;break;
//			case 51:control_data.Gravity_compensation = 0.1;break;
//			case 52:control_data.Gravity_compensation = 0.1;break;
//			case 53:control_data.Gravity_compensation = 0.1;break;
//			case 54:control_data.Gravity_compensation = 0.1;break;
//			case 55:control_data.Gravity_compensation = 0.17;break;
//			case 56:control_data.Gravity_compensation = 0.17;break;
//			case 57:control_data.Gravity_compensation = 0.17;break;
//			case 58:control_data.Gravity_compensation = 0.17;break;
//			case 59:control_data.Gravity_compensation = 0.17;break;
//			case 60:control_data.Gravity_compensation = 0.17;break;
//			case 61:control_data.Gravity_compensation = 0.17;break;
//			case 62:control_data.Gravity_compensation = 0.17;break;
//			case 63:control_data.Gravity_compensation = 0.17;break;
//			case 64:control_data.Gravity_compensation = 0.17;break;
//			case 65:control_data.Gravity_compensation = 0.17;break;
//			case 66:control_data.Gravity_compensation = 0.17;break;
//			case 67:control_data.Gravity_compensation = 0.17;break;
//			case 68:control_data.Gravity_compensation = 0.17;break;
//			case 69:control_data.Gravity_compensation = 0.17;break;
//			default:break;
//		}
//		
////if(Gravity_distance>0&&Gravity_distance<40)
////{
//		control_data.Gravity_compensation=0.004*Gravity_distance;
////}
		switch (Gravity_distance)
		{
			case 1:control_data.Gravity_compensation = 0.02;break;
			case 2:control_data.Gravity_compensation = 0.02;break;
			case 3:control_data.Gravity_compensation = 0.02;break;
			case 4:control_data.Gravity_compensation = 0.02;break;
			case 5:control_data.Gravity_compensation = 0.02;break;
			case 6:control_data.Gravity_compensation = 0.02;break;
			case 7:control_data.Gravity_compensation = 0.02;break;
			case 8:control_data.Gravity_compensation = 0.02;break;
			case 9:control_data.Gravity_compensation = 0.02;break;
			case 10:control_data.Gravity_compensation = 0.04;break;
			case 11:control_data.Gravity_compensation = 0.04;break;
			case 12:control_data.Gravity_compensation = 0.04;break;
			case 13:control_data.Gravity_compensation = 0.04;break;
			case 14:control_data.Gravity_compensation = 0.04;break;
			case 15:control_data.Gravity_compensation = 0.04;break;
			case 16:control_data.Gravity_compensation = 0.09;break;
			case 17:control_data.Gravity_compensation = 0.09;break;
			case 18:control_data.Gravity_compensation = 0.09;break;
			case 19:control_data.Gravity_compensation = 0.09;break;
			case 20:control_data.Gravity_compensation = 0.09;break;
			case 21:control_data.Gravity_compensation = 0.11;break;
			case 22:control_data.Gravity_compensation = 0.11;break;
			case 23:control_data.Gravity_compensation = 0.11;break;
			case 24:control_data.Gravity_compensation = 0.11;break;
			case 25:control_data.Gravity_compensation = 0.11;break;
			case 26:control_data.Gravity_compensation = 0.12;break;
			case 27:control_data.Gravity_compensation = 0.12;break;
			case 28:control_data.Gravity_compensation = 0.12;break;
			case 29:control_data.Gravity_compensation = 0.12;break;
			case 30:control_data.Gravity_compensation = 0.12;break;
			case 31:control_data.Gravity_compensation = 0.14;break;
			case 32:control_data.Gravity_compensation = 0.14;break;
			case 33:control_data.Gravity_compensation = 0.14;break;
			case 34:control_data.Gravity_compensation = 0.14;break;
			case 35:control_data.Gravity_compensation = 0.14;break;
			case 36:control_data.Gravity_compensation = 0.14;break;
			case 37:control_data.Gravity_compensation = 0.14;break;
			case 38:control_data.Gravity_compensation = 0.14;break;
			case 39:control_data.Gravity_compensation = 0.14;break;
			case 40:control_data.Gravity_compensation = 0.14;break;
			case 41:control_data.Gravity_compensation = 0.16;break;
			case 42:control_data.Gravity_compensation = 0.16;break;
			case 43:control_data.Gravity_compensation = 0.16;break;
			case 44:control_data.Gravity_compensation = 0.16;break;
			case 45:control_data.Gravity_compensation = 0.16;break;
			case 46:control_data.Gravity_compensation = 0.16;break;
			case 47:control_data.Gravity_compensation = 0.16;break;
			case 48:control_data.Gravity_compensation = 0.16;break;
			case 49:control_data.Gravity_compensation = 0.16;break;
			case 50:control_data.Gravity_compensation = 0.16;break;
			case 51:control_data.Gravity_compensation = 0.16;break;
			case 52:control_data.Gravity_compensation = 0.16;break;
			case 53:control_data.Gravity_compensation = 0.16;break;
			case 54:control_data.Gravity_compensation = 0.16;break;
			case 55:control_data.Gravity_compensation = 0.16;break;
			case 56:control_data.Gravity_compensation = 0.18;break;
			case 57:control_data.Gravity_compensation = 0.18;break;
			case 58:control_data.Gravity_compensation = 0.18;break;
			case 59:control_data.Gravity_compensation = 0.18;break;
			case 60:control_data.Gravity_compensation = 0.18;break;
			case 61:control_data.Gravity_compensation = 0.18;break;
//			case 62:control_data.Gravity_compensation = 0.3;break;
//			case 63:control_data.Gravity_compensation = 0.3;break;
//			case 64:control_data.Gravity_compensation = 0.3;break;
//			case 65:control_data.Gravity_compensation = 0.3;break;
//			case 66:control_data.Gravity_compensation = 0.3;break;
//			case 67:control_data.Gravity_compensation = 0.3;break;
//			case 68:control_data.Gravity_compensation = 0.26;break;
//			case 69:control_data.Gravity_compensation = 0.26;break;
			default: control_data.Gravity_compensation = 0.11; break;
		}
//		 double a = 9.8 * 9.8 * 0.25;
//    double b = 14 * 14 - Gravity_distance * 9.8 * cos(1.5707963267948966192313216916398 + control_data.pitch_dev);
//    double c = Gravity_distance * Gravity_distance;
//    // ���������ʽ�����t^2
//    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
////    std::cout << fmt::format("a:{}, b:{}, c:{}", a, b, c) << std::endl;
////    std::cout << "t2:" << t_2 << std::endl;
//    double fly_time = sqrt(t_2);                                       // �ӵ�����ʱ�䣨��λ:s��
//    // ���̧ǹ�߶ȣ����ӵ���׹�߶�
//    double height = 0.5 * 9.8 * t_2;
//     control_data.Gravity_compensation=height+54;
SortAver_Filter(control_data.Gravity_compensation,&GG,25);  //ȥ��ֵƽ���˲�
		switch(Judge_data.shooter_id1_42mm_speed_limit)
	{
		case 15:vision_G=15; break;
		case 18:vision_G=6; break;
		case 22:vision_G=4; break;
		case 30:vision_G=2; break;  //-6 fc
		default:  break;
	}
	 control_data.G_dev = GG*vision_G;
	 //control_data.G_dev = control_data.Gravity_compensation;
//	}
}
//������׹
float dropOffset(float distance, float shootSpd) //��׹������
{
	if (distance >= shootSpd * shootSpd / 5.0f)
		return 0.0f;
	else if (distance <= 0)
		return 0.0f;
	else
	{
		return asinf(distance * 9.8f / (shootSpd * shootSpd)) * 28.65f - 7.568f * exp(-0.5633f * distance);
	}
}
/**
  * @brief        Ŀ���ٶȼ���
  * @author         
  * @param[in]    Ԥ��Ŀ����ٶȡ���ǰʱ�䡢Ԥ��Ŀ��ʱǹ��
  * @retval		  Ԥ���Ŀ���ٶ�
  * @note  		
  */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;
	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2; //�����ٶ�
		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}
	if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0; //ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed; //��������ٶ�
}
/***************����******************/
//���
float Auto_Error_Yaw[2]; //    now/last
float Auto_Error_Pitch[2];
float Auto_Distance; //���뵥Ŀ
//����ͻȻ����,�������˲�������ʱ
uint16_t Auto_KF_Delay = 0;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch; //�������˲��ٶȲ���ֵ
float *yaw_kf_result, *pitch_kf_result;					//���׿������˲����,0�Ƕ� 1�ٶ�
vision_KM_debug_data_t vision_KM;
/**
  * @brief  ��̨������ʼ��
  * @param  void
  * @retval void
  * @attention û�м�I�������,ֻ��ϵͳ����ʱ����һ��
  */
void GIMBAL_InitArgument(void)
{
/*--------------����ǶȲ�����ʼ��----------------*/
#if INFANTRY_DEBUG_ID == DEBUG_ID_ONE

	debug_y_sk = 88; //38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_y_sb_sk = 10;
	debug_p_sk = 26;	   //�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_auto_err_y = 16; //15;//10;//15;//�Ƕȹ���ر�Ԥ��
	debug_auto_err_p = 8;
	debug_kf_delay = 180;		//100;//200;//120;//150;//Ԥ����ʱ����
	debug_kf_speed_yl = 0.38;	//0.1;//0.2;//0.1;//0.08;//0.1;//0.3;//�ٶȹ��͹ر�Ԥ��
	debug_kf_speed_yh = 6;		//�ٶȹ��߹ر�Ԥ��
	debug_kf_speed_yl_sb = 0.1; //
	debug_kf_speed_pl = 0.1;	//pitch�ٶȹ��͹ر�Ԥ��
	debug_kf_y_angcon = 260;	//125;//115;//135;//Ԥ�����޷�
	debug_kf_p_angcon = 45;		//pitchԤ�����޷�

	//���
	//����
	debug_y_mid = Mech_Mid_Yaw;
	debug_p_mid = Mech_Mid_Pitch;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//��̨
	Buff_Pitch_Comp_Gimbal = 0;
	Buff_Yaw_Comp_Gimbal = 0;

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 1;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = 0;

#elif INFANTRY_DEBUG_ID == DEBUG_ID_TWO

	debug_y_sk = 52;		 //45;//14.8;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_y_sb_sk = 62;		 //61;//55;
	debug_y_sb_brig_sk = 88; //
	debug_p_sk = 20;		 //�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_auto_err_y = 120;	 //�Ƕȹ���ر�Ԥ��
	debug_auto_err_p = 150;
	debug_kf_delay = 80;		//Ԥ����ʱ����
	debug_kf_speed_yl = 0.2;	//0.35;//�ٶȹ��͹ر�Ԥ��
	debug_kf_speed_yl_sb = 0.2; //
	debug_kf_speed_yh = 4.2;	//�ٶȹ��߹ر�Ԥ��
	debug_kf_speed_pl = 0.15;	//pitch�ٶȹ��͹ر�Ԥ��
	debug_kf_y_angcon = 220;	//125;//115;//135;//Ԥ�����޷�
	debug_kf_p_angcon = 45;		//pitchԤ�����޷�

	//���
	//����
	debug_y_mid = 4122; //5883;
	debug_p_mid = 3860; //3565;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//��̨
	//7.1��
	Buff_Pitch_Comp_Gimbal = -78; //-88;//����-88��������-96
	//8��
	//Buff_Pitch_Comp_Gimbal = -86;//-96;
	Buff_Yaw_Comp_Gimbal = -8; //28m/s

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 0.99;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = -20;

#elif INFANTRY_DEBUG_ID == DEBUG_ID_THREE

	debug_y_sk = 45; //14.8;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_y_sb_sk = 55;
	debug_y_sb_brig_sk = 90; //
	debug_p_sk = 20;		 //�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	debug_auto_err_y = 120;	 //�Ƕȹ���ر�Ԥ��
	debug_auto_err_p = 150;
	debug_kf_delay = 80;		//Ԥ����ʱ����
	debug_kf_speed_yl = 0.2;	//�ٶȹ��͹ر�Ԥ��
	debug_kf_speed_yl_sb = 0.2; //
	debug_kf_speed_yh = 4;		//�ٶȹ��߹ر�Ԥ��
	debug_kf_speed_pl = 0.15;	//pitch�ٶȹ��͹ر�Ԥ��
	debug_kf_y_angcon = 220;	//125;//115;//135;//Ԥ�����޷�
	debug_kf_p_angcon = 45;		//pitchԤ�����޷�

	//���
	//����
	debug_y_mid = Mech_Mid_Yaw;
	debug_p_mid = Mech_Mid_Pitch;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//��̨
	//7.1��
	Buff_Pitch_Comp_Gimbal = -22; //-30;//-24;//-20;//-10;//����-24��������-30
	//8��
	//Buff_Pitch_Comp_Gimbal = -30;

	Buff_Yaw_Comp_Gimbal = -13; //19;

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 1;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = -13;
#elif INFANTRY_DEBUG_ID == DEBUG_ID_FOUR

	vision_KM.debug_y_sk = 50;		   //45;//35;//14.8;//yaw�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
	vision_KM.debug_y_sb_sk = 59;	   //55;         //�ڱ���
	vision_KM.debug_y_sb_brig_sk = 90; //       //�ڱ���
	vision_KM.debug_p_sk = 20;		   //pitch�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��

	vision_KM.debug_auto_err_y = 120; //yaw�Ƕȹ���ʱ�ر�Ԥ��
	vision_KM.debug_auto_err_p = 150; //pitch�Ƕȹ���ʱ�ر�Ԥ��

	vision_KM.debug_kf_delay = 80;		  //Ԥ����ʱ����
	vision_KM.debug_kf_speed_yl = 0.2;	  //0.35;  //yaw�ٶȹ��͹ر�Ԥ��
	vision_KM.debug_kf_speed_yl_sb = 0.2; //0.2;//�ڱ���
	vision_KM.debug_kf_speed_yh = 5;	  //yaw�ٶȹ��߹ر�Ԥ��
	vision_KM.debug_kf_speed_pl = 0.15;	  //pitch�ٶȹ��͹ر�Ԥ��
	vision_KM.debug_kf_y_angcon = 220;	  //125;//115;//135;//yawԤ�����޷�
	vision_KM.debug_kf_p_angcon = 45;	  //pitchԤ�����޷�
#endif

	//�������˲�����ʼ��

	/*���鿨�����˲�,����*/
	mat_init(&yaw_kalman_filter.Q, 2, 2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R, 2, 2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

	mat_init(&pitch_kalman_filter.Q, 2, 2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R, 2, 2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
}
/**********************************************************************************/
/** 
  * @brief  ������ƺ���
  * @param  void
  * @retval void
  * @attention �м�����(0,0),�����Ҹ�,�ϸ�����
  *            yawΪ������ģʽ,pitchΪ��еģʽ(��ʵpitchȫ�̶����û�еģʽ)
  *            ֻ�е����������˲��ڵ�ǰʵʱ�Ƕ����ۼ�����Ŀ��Ƕ�
  *            ������һ��,Ŀ��ǶȾ�ʵʱ����
  */
uint8_t view_control_flag = 0;
float debug_y_dk = 450;	  //yaw����Ԥ�������Խ��Ԥ��Խ��
uint32_t Vision_Time[2];  // NOW/LAST
int vision_time_js;		  //�����Ӿ��ӳ�
float error_yaw_k = 1;	  //7.5;//5.6;//2.2;//���Ŵ�
float error_pitch_k = 10; //5;//3;//2.1;//���Ŵ�
float debug_kf_y_angle;	  //yawԤ���ݴ�
float debug_kf_p_angle;	  //pitchԤ���ݴ�
//���ݾ������Ԥ��������޷�
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;
float pitch_speed_k = 0;
float kf_pitch_angcon = 0;
float debug_kf_angle_temp;		//Ԥ��Ƕ�б���ݴ���
float debug_kf_angle_ramp = 20; //Ԥ��Ƕ�б�±仯��
float debug_kf_dist;
float debug_dist_bc = 0;
float gim_auto_ramp_y = 5; //10;//�տ�������ʱ�����ƹ�ȥ����ֹ�Ӿ���Ӱ��֡
float gim_auto_ramp_p = 5; //�տ�������ʱ�����ƹ�ȥ����ֹ�Ӿ���Ӱ��֡
float kf_speed_yl = 0;	   //
void GIMBAL_AUTO_Mode_Ctrl(gimbal_control_data_t *gimbal_control_data)
{
	static float yaw_angle_raw, pitch_angle_raw; //�������˲��ǶȲ���ֵ
	static float yaw_angle_ref;					 //��¼Ŀ��Ƕ�
	static float pitch_angle_ref;				 //��¼Ŀ��Ƕ�
	float kf_delay_open = 0;
	kf_speed_yl = vision_KM.debug_kf_speed_yl; //yaw�ٶȹ��͹ر�Ԥ��

	/*�����������������������������������������������ݸ��¡�������������������������������������������������������������*/
	if (view_control_flag == 1) //�Ӿ����ݸ�����
	{
		//����Ŀ��Ƕ�//��¼��ǰʱ�̵�Ŀ��λ��,Ϊ��������׼��
		yaw_angle_ref = (gimbal_control_data->gimbal_INS->yaw_angle - (control_data.yaw_dev));
		pitch_angle_ref = (gimbal_control_data->gimbal_INS->roll_angle + (control_data.pitch_dev * 0.2f));
		view_control_flag = 0;					//һ��Ҫ�ǵ�����,�����һֱִ��
		Vision_Time[NOW] = xTaskGetTickCount(); //��ȡ�����ݵ���ʱ��ʱ��
	}
	/*���������������������������������������������������ݸ��¡���������������������������������������������������������*/

	/*�����������������������������������������������׿����������������������������������������������������������������������*/
	if (Vision_Time[NOW] != Vision_Time[LAST]) //���������ݵ�����ʱ��
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];
		//���¶��׿������˲�����ֵ
		yaw_angle_raw = yaw_angle_ref;
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	//Ŀ���ٶȽ���
	if (control_data.frame_seq == 4) //ʶ����Ŀ��
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);
	}
	else
	{
		//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), gimbal_control_data->gimbal_INS->yaw_angle);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), gimbal_control_data->gimbal_INS->roll_angle);
		//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, gimbal_control_data->gimbal_INS->yaw_angle, 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, gimbal_control_data->gimbal_INS->roll_angle, 0);
	}
	//δʶ��Ŀ��ʱ�������������̨
	if (control_data.frame_seq == 4) //ʶ����Ŀ��     //δʶ����ô����4��   *hyj
	{
		Auto_KF_Delay++; //�˲���ʱ����
						 //		if(VisionRecvData.auto_too_close == TRUE
						 //			&& (Chassis_IfCORGI() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE) )//Ŀ�����̫������СԤ��
						 //		{
						 //			yaw_speed_k = vision_KM.debug_y_sk;///4.f;//3.f;//Ԥ��ϵ������
						 //			kf_yaw_angcon = vision_KM.debug_kf_y_angcon;//3.f;//2.f;
						 //			kf_speed_yl = vision_KM.debug_kf_speed_yl;
						 //		}
						 //		else//����Ԥ����
						 //		{
						 //			if( GIMBAL_AUTO_PITCH_SB() == TRUE )  //���ݽǶ��ж��Ƿ��ڴ��ڱ� ��һ�����˿�����ʱ���Լ�����
						 //			{
						 //				yaw_speed_k = vision_KM.debug_y_sb_sk;
						 //				kf_yaw_angcon = vision_KM.debug_kf_y_angcon;
						 //				kf_speed_yl = vision_KM.debug_kf_speed_yl_sb;
						 //
						 //				if(IF_KEY_PRESSED_G)  //���ߵ�ʱ������������ �������ھͿ������ִ��ڱ���
						 //				{
						 //					yaw_speed_k = vision_KM.debug_y_sb_brig_sk;
						 //					kf_yaw_angcon = vision_KM.debug_kf_y_angcon*1.1f;
						 //					kf_speed_yl = vision_KM.debug_kf_speed_yl_sb*0.4f;//0.9f;
						 //				}
						 //			}
		if (1)			 //һֱ�����������
		{
			yaw_speed_k = vision_KM.debug_y_sk;
			kf_yaw_angcon = vision_KM.debug_kf_y_angcon;
			kf_speed_yl = vision_KM.debug_kf_speed_yl;
		}
		//		}
		/*���������������������������������������������������׿������������������������������������������������������������������*/

		/*Ԥ�⿪������*/
		if (fabs(Auto_Error_Yaw[NOW]) < vision_KM.debug_auto_err_y																							 //debug��
			&& Auto_KF_Delay > kf_delay_open && fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl && fabs(yaw_kf_result[KF_SPEED]) < vision_KM.debug_kf_speed_yh) //��yaw�ǶȲ��� �ٶȲ���Ҳ����ʱ���Կ�Ԥ��
		{
			if (yaw_kf_result[KF_SPEED] >= 0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1; //debug_kf_dist;
			}
			else if (yaw_kf_result[KF_SPEED] < 0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1; //debug_kf_dist;
			}
			//			debug_kf_angle_temp = vision_KM.debug_y_sk * yaw_kf_result[KF_SPEED];//�˴�����Ҫ�ٶ�̫����Ԥ��
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -vision_KM.debug_kf_y_angcon, vision_KM.debug_kf_y_angcon); //Ԥ���ݴ����޷�
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);							   //ʹԤ���������仯
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -vision_KM.debug_kf_y_angcon, vision_KM.debug_kf_y_angcon);

			gimbal_control_data->gimbal_yaw_set = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle; //vision_KM.debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
		}
		/*Ԥ������û�ﵽ���ر�Ԥ��*/
		else
		{
			gimbal_control_data->gimbal_yaw_set = yaw_angle_ref;
			//			if( fabs(Auto_Error_Yaw[NOW]) < 1.5f )//�ӽ�Ŀ��
			//			{
			//				mobpre_yaw_stop_delay++;
			//				if(mobpre_yaw_stop_delay > 25)//ֹͣ�ȶ�50ms
			//				{
			//
			//				}
			//			}
		}
		/*---------------pitch������С��Ԥ��------------------*/
		if (Auto_KF_Delay > vision_KM.debug_kf_delay && fabs(Auto_Error_Pitch[NOW]) < vision_KM.debug_auto_err_p && fabs(pitch_kf_result[KF_SPEED]) > vision_KM.debug_kf_speed_pl
			// && (GIMBAL_AUTO_PITCH_SB_SK() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE) //�Ƿ��ж̾������ڱ�    //*hyj    ��ע��
			// 	&& VisionRecvData.distance/100 < 4.4f                                                            //�Ⱦ���     ��ע��
		)
		{
			if (1) //(VisionRecvData.auto_too_close == TRUE)//Ŀ�����̫������СԤ��                                                  //��ע��
			{
				pitch_speed_k = vision_KM.debug_p_sk / 2.f; //Ԥ��ϵ������
				kf_pitch_angcon = vision_KM.debug_kf_p_angcon / 1.5f;
			}
			else //����Ԥ����
			{
				pitch_speed_k = vision_KM.debug_p_sk;
				kf_pitch_angcon = vision_KM.debug_kf_p_angcon;
			}
			if (pitch_kf_result[KF_SPEED] >= 0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - vision_KM.debug_kf_speed_pl);
			}
			else
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + vision_KM.debug_kf_speed_pl);
			}
			//pitchԤ�����޷�
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);

			gimbal_control_data->gimbal_pitch1_set = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle; //һ��pitch:gimbal_control_data->gimbal_pitch_set
		}
		/*Ԥ������û�ﵽ���ر�Ԥ��*/
		else
		{
			gimbal_control_data->gimbal_pitch1_set = pitch_angle_ref; //һ��pitch:gimbal_control_data->gimbal_pitch_set
		}
	} 
	else //δʶ��Ŀ��,�����������̨ �������С����
	{
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, control_data.yaw_dev, 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, control_data.pitch_dev, 0);
	}
}

/**
  * @brief  ������ƺ���
  * @param  void
  * @retval 
  * @attention	�������Ӿ�ģʽ�У�һ���Ӿ���־��ʧ��תΪ����ģʽ   ˫pitch  II
  */
void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,
							   robot_control_mode_e robot_control_mode,
							   _tx2_control_data control_data)
{
	if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO) //mpu
	{
		if (robot_control_mode == KEY_MOUSE_MODE) //KEY_MOUSE_MODE
		{
			if (shoot_control_data.shoot_vision_flag && view_control_flag) //��F���Ӿ�����     *hyj   ���Ͽ������˲�
			{
				gimbal_control_data->gimbal_yaw_set = (gimbal_control_data->gimbal_INS->yaw_angle + control_data.yaw_compensate - (control_data.yaw_dev)); // gimbal_control_data->gimbal_pitch_set = (gimbal_control_data->gimbal_INS->roll_angle + (control_data.pitch_dev*0.2f));
				// GIMBAL_AUTO_Mode_Ctrl(gimbal_control_data);
				// gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y, \
				// 				-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
				// 				MOUSE_SINGLE_TIME_ALLOW_VALUE);
				// gimbal_control_data->gimbal_pitch_set += (gimbal_control_data->rc_ctrl->mouse.y) * \
				// 										gimbal_control_data->rc_ctrl->pitch_sensit;
				view_control_flag = 0; //����Ӿ����ݱ�־λ
			}
			else
			{
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.x,
								 -MOUSE_SINGLE_TIME_ALLOW_VALUE,
								 MOUSE_SINGLE_TIME_ALLOW_VALUE);
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y,
								 -MOUSE_SINGLE_TIME_ALLOW_VALUE,
								 MOUSE_SINGLE_TIME_ALLOW_VALUE);
				mouse_sensit_cali(gimbal_control_data->rc_ctrl);

				gimbal_control_data->gimbal_yaw_set += (gimbal_control_data->rc_ctrl->mouse.x) *
													   		gimbal_control_data->rc_ctrl->yaw_sensit;


				gimbal_control_data->gimbal_pitch1_set1-=  (gimbal_control_data->rc_ctrl->mouse.y) *
															gimbal_control_data->rc_ctrl->pitch_sensit ; 
				gimbal_control_data->gimbal_pitch1_set = gimbal_control_data->gimbal_pitch1_set1+90;
				

				gimbal_control_data->gimbal_pitch2_set1 -= (gimbal_control_data->rc_ctrl->mouse.y) *
															gimbal_control_data->rc_ctrl->pitch_sensit; 
				gimbal_control_data->gimbal_pitch2_set = gimbal_control_data->gimbal_pitch2_set1+90;
			}
		}
		// else if(robot_control_mode != KEY_MOUSE_MODE) //REMOTE_MODE		�����Ӿ���
		else if (robot_control_mode == REMOTE_MODE) //REMOTE_MODE
		{
			
			//�Ӿ�����     *hyj
			if (view_control_flag == 1)
			{
				vision_install_compensate();//�Ӿ�ͬλ�ò���
				// vision_Gravity_compensate();
				vision_INS_distance_compensation(&control_data,gimbal_control_data);
				
				gimbal_control_data->gimbal_yaw_set = (
																		-gimbal_control_data->gimbal_INS->yaw_angle +  //��ǰ��̬��
																			control_data.yaw_compensate + 								//�Ӿ�ͬλ�ò���
																		(control_data.yaw_dev*yaw_vision_Kp)+					//λ��
																		constrain_float(yaw_vision_Kd*control_data.yaw_dev_err,-1,1)+	//λ�����
																		control_data.yaw_INS_compensation+
																		YAW_FOLLOW_CLOSE);;														
	
				gimbal_control_data->gimbal_pitch1_set1 = (-(gimbal_control_data->gimbal_INS->roll_angle) +
																		control_data.pitch_compensate + 
																		control_data.pitch_dev * pitch_vision_Kp)   +
																		constrain_float(pitch_vision_Kd*control_data.pitch_dev_err,-1,1)+
																		control_data.pitch_distance_compensation+
																		PITCH_FOLLOW_CLOSE;
				gimbal_control_data->gimbal_pitch1_set =(gimbal_control_data->gimbal_pitch1_set1+90);
				
				gimbal_control_data->gimbal_pitch2_set1 = (-(gimbal_control_data->gimbal_INS->roll_angle) +
																		control_data.pitch_compensate + 
																		control_data.pitch_dev * pitch_vision_Kp)   +
																		constrain_float(pitch_vision_Kd*control_data.pitch_dev_err,-1,1)+
																		control_data.pitch_distance_compensation+
																		PITCH_FOLLOW_CLOSE;																		
				gimbal_control_data->gimbal_pitch2_set = gimbal_control_data->gimbal_pitch2_set1+90;
				

				

				 view_control_flag = 0;
			} 
			else 
			{
				gimbal_control_data->gimbal_yaw_set += (gimbal_control_data->rc_ctrl->rc.ch0) *
													   GAMBAL_YAW_MAX_ANGLE_SET_FACT;

				gimbal_control_data->gimbal_pitch1_set1+= (gimbal_control_data->rc_ctrl->rc.ch1) *
															  GAMBAL_PITCH_MAX_ANGLE_SET_FACT ; //ch1 ˫Pitch ��ң�ؿ���������
		 		gimbal_val_limit_float((&gimbal_control_data->gimbal_pitch1_set1),
								 		-45.0, 
								 		45.0);//�Ƕ��޷� 
				gimbal_control_data->gimbal_pitch1_set = gimbal_control_data->gimbal_pitch1_set1+90;
				

				gimbal_control_data->gimbal_pitch2_set1 += (gimbal_control_data->rc_ctrl->rc.ch1) *
															  GAMBAL_PITCH_MAX_ANGLE_SET_FACT; //ch1 ˫Pitch
				gimbal_val_limit_float((&gimbal_control_data->gimbal_pitch2_set1),
								 		-45.0,
								 		45.0);
				gimbal_control_data->gimbal_pitch2_set = gimbal_control_data->gimbal_pitch2_set1+90;
			}
		}
		gimbal_control_data->gimbal_yaw_fdb = -(gimbal_control_data->gimbal_INS->yaw_angle);

		// gimbal_control_data->gimbal_pitch1_fdb = gimbal_control_data->gimbal_pitch_motor_msg->encoder.ecd_angle-2;
 		// gimbal_control_data->gimbal_pitch2_fdb = gimbal_control_data->gimbal_pitch1_motor_msg->encoder.ecd_angle;//������

		gimbal_control_data->gimbal_pitch1_fdb = -( gimbal_control_data->gimbal_INS->roll_angle); //2 pitch
		gimbal_control_data->gimbal_pitch2_fdb = -( gimbal_control_data->gimbal_INS->roll_angle); //2 pitch //������
	}

	else if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_ENCONDE) //��ʼ��ʱ�ı���ģʽ set ���޸�
	{
		// ������������̨
		// gimbal_control_data->gimbal_yaw_set = -gimbal_control_data->gimbal_INS->yaw_angle;
		// ����ʱ��̨�ҵ��̣���ȫ��һ
		gimbal_control_data->gimbal_yaw_set =-( gimbal_control_data->gimbal_INS->yaw_angle  \
																					+(((float)(GAMBAL_YAW_INIT_ENCODE_VALUE_OU-gimbal_control_data->gimbal_yaw_motor_msg->encoder.raw_value))/GAMBAL_YAW_angle_VALUE));

		gimbal_control_data->gimbal_pitch1_set = gimbal_control_data->gimbal_INS->roll_angle \
																					- ((float)(GAMBAL_PITCH_INIT_ENCODE_VALUE - gimbal_control_data->gimbal_pitch_motor_msg->encoder.raw_value)) / GAMBAL_PITCH_angle_VALUE; //2 pitch
		gimbal_control_data->gimbal_pitch2_set = gimbal_control_data->gimbal_INS->roll_angle \
																					- ((float)(GAMBAL_PITCH_INIT_ENCODE_VALUE - gimbal_control_data->gimbal_pitch_motor_msg->encoder.raw_value)) / GAMBAL_PITCH_angle_VALUE; //2 pitch

		gimbal_control_data->gimbal_yaw_fdb = -gimbal_control_data->gimbal_INS->yaw_angle;
 

		gimbal_control_data->gimbal_pitch1_fdb = gimbal_control_data->gimbal_INS->roll_angle; //2 pitch
		gimbal_control_data->gimbal_pitch2_fdb = gimbal_control_data->gimbal_INS->roll_angle; //2 pitch
		

	}
	//Pitch�޷�
	if (gimbal_control_data->gimbal_pitch1_set > 110)
	{
		gimbal_control_data->gimbal_pitch1_set = 110;
	}
	else if (gimbal_control_data->gimbal_pitch1_set < 65)
	{
		gimbal_control_data->gimbal_pitch1_set = 65;
	}
	if (gimbal_control_data->gimbal_pitch2_set > 110)
	{
		gimbal_control_data->gimbal_pitch2_set = 110;
	}
	else if (gimbal_control_data->gimbal_pitch2_set < 65)
	{
		gimbal_control_data->gimbal_pitch2_set = 65;
	}

}



void gimbal_set_motor_feedback_mode(gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
	gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
}
/** 
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
/*******************************2��pitch   II**************************************/
void gimbal_cascade_pid_calculate(gimbal_pid_t *gimbal_pid,
								  gimbal_control_data_t *gimbal_control_data)
{
	gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.fdb = gimbal_control_data->gimbal_yaw_fdb;
	gimbal_pid->yaw_pid.position_pid.Calc(&gimbal_pid->yaw_pid.position_pid);

	gimbal_pid->yaw_pid.speed_pid.set = gimbal_pid->yaw_pid.position_pid.output; ///���⻷λ�������Ϊ�ڻ��ٶ�����
	if (gimbal_control_data->yaw_motor_fdb_mode == GIMBAL_MOTOR_GYRO || 1)
	{
		gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y; ///������ԭʼ������Ϊ�ڻ��ٶ�����
	}
	else
	{
		gimbal_pid->yaw_pid.speed_pid.fdb = 0;
	}
	gimbal_pid->yaw_pid.speed_pid.Calc(&gimbal_pid->yaw_pid.speed_pid);

	//pitch1
	gimbal_pid->pitch1_pid.position_pid.set = gimbal_control_data->gimbal_pitch1_set; //����ֵ�����̶�ֵ����ֵ�������set��fdb��������
	gimbal_pid->pitch1_pid.position_pid.fdb = gimbal_control_data->gimbal_pitch1_fdb; //���������ǵĽǶ�ֵ��ͬʱ������������Ļ�������������ĸ˻᲻��ϣ���
	gimbal_pid->pitch1_pid.position_pid.Calc(&gimbal_pid->pitch1_pid.position_pid);

	gimbal_pid->pitch1_pid.speed_pid.set = gimbal_pid->pitch1_pid.position_pid.output;
	if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO || 1)
	{
		gimbal_pid->pitch1_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_x;
	}
	else
	{
		gimbal_pid->pitch1_pid.speed_pid.fdb = 0;
	}
	gimbal_pid->pitch1_pid.speed_pid.Calc(&gimbal_pid->pitch1_pid.speed_pid);
	//pitch2
	gimbal_pid->pitch2_pid.position_pid.set = gimbal_control_data->gimbal_pitch2_set;
	gimbal_pid->pitch2_pid.position_pid.fdb = gimbal_control_data->gimbal_pitch2_fdb;
	gimbal_pid->pitch2_pid.position_pid.Calc(&gimbal_pid->pitch2_pid.position_pid);

	gimbal_pid->pitch2_pid.speed_pid.set = gimbal_pid->pitch2_pid.position_pid.output;
	if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO || 1)
	{
		gimbal_pid->pitch2_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_x;
	}
	else
	{
		gimbal_pid->pitch2_pid.speed_pid.fdb = 0;
	}
	gimbal_pid->pitch2_pid.speed_pid.Calc(&gimbal_pid->pitch2_pid.speed_pid);
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
/********************************2��pitch  II***************************************/
void gimbal_control_loop(gimbal_pid_t *gimbal_pid,
						 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->given_current.yaw_motor = -gimbal_pid->yaw_pid.speed_pid.output; //g6020����װ��Ҫ�Ӹ���
	gimbal_control_data->given_current.pitch1_motor = gimbal_pid->pitch1_pid.speed_pid.output;
	gimbal_control_data->given_current.pitch2_motor = gimbal_pid->pitch2_pid.speed_pid.output;
  
	if (get_robot_control_mode() == GUI_CALI_MODE)
	{
		set_gimbal_stop();
	}
	else
	{
		set_gimbal_behaviour(gimbal_control_data->given_current.yaw_motor,
							 gimbal_control_data->given_current.yaw_motor,
							 gimbal_control_data->given_current.pitch1_motor,
							 gimbal_control_data->given_current.pitch2_motor);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;

	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;

	pid->mode = cali_pid->mode;

	pid->Calc = &PID_Calc;
	pid->Reset = &PID_Reset;
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_init(gimbal_pid_t *gimbal_pid,
				 cali_gimbal_t *cali_pid,
				 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->rc_ctrl = get_rc_data_point();
	gimbal_control_data->gimbal_INS = get_INS_point();
	gimbal_control_data->gimbal_yaw_motor_msg = get_yaw_motor_msg_point();
	gimbal_control_data->gimbal_pitch_motor_msg = get_pitch_motor_msg_point();
	gimbal_control_data->gimbal_pitch1_motor_msg = get_pitch1_motor_msg_point();

	// gimbal_control_data->gimbal_pitch_motor_msg = get_pitch1_motor_msg_point();
	//yaw cascade pid ///У׼���yaw�ǶȺ��ٶȸ�����̨
	gimbal_pid_init(&gimbal_pid->yaw_pid.position_pid, &cali_pid->yaw_pid.position); //���渳��ǰ��
	gimbal_pid_init(&gimbal_pid->yaw_pid.speed_pid, &cali_pid->yaw_pid.speed);
	//pitch cascade pid
	/*******************����pitch*******************************/
	gimbal_pid_init(&gimbal_pid->pitch1_pid.position_pid, &cali_pid->pitch1_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch1_pid.speed_pid, &cali_pid->pitch1_pid.speed);
	gimbal_pid_init(&gimbal_pid->pitch2_pid.position_pid, &cali_pid->pitch2_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch2_pid.speed_pid, &cali_pid->pitch2_pid.speed);

	// set_robot_control_mode(GUI_CALI_MODE);
	// set_robot_work_mode(ROBOT_CALI_MODE);
	set_gimbal_work_mode(GIMBAL_CALI_MODE);
	/*���ø��ϵ������˵Ĺ���ģʽ����Ҫ���ڰ������ģʽ���ϵ�*/
	set_robot_control_mode(REMOTE_MODE);
	set_robot_work_mode(ROBOT_COMMON_MODE); //��ͨ���̸���ģʽ
}
/**
  * @brief        ����ʱ����GUI����
  * @author         
  * @param[in]      
  * @retval			
  * @note         ///У׼ģʽ�»�ȡң����ģ���ֵ  
  */

extern TaskHandle_t GUI_Task_Handler;
void set_GUI_task_state(void)
{
	if (get_robot_control_mode() == GUI_CALI_MODE)
	{
		if (eTaskGetState(GUI_Task_Handler) == eSuspended)
		{
			vTaskResume(GUI_Task_Handler);	   //�ָ�����
			HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); //������ʱ����ȡrcģ�⽡ֵ
			LED_Fill(0x00);
			oled_init();
			LED_Fill(0x00);
		}
	}
	else
	{
		if (eTaskGetState(GUI_Task_Handler) != eSuspended)
		{

			HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
			vTaskSuspend(GUI_Task_Handler);
			LASER_OFF();
			LED_Fill(0x00);
			oled_init();
			LED_Fill(0x00);
		}
	}
	if (get_robot_control_mode() == GUI_CALI_MODE)
	{

		REDLED_ON(); ///��Ʊ�־λ
	}
	else
	{

		REDLED_OFF();
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval whileѭ����ǰ��������������4%(���е�һ��0.3%) ����while����15%
  */
void gimbal_task(void *argument)
{
	TickType_t current_time = 0;
	vTaskDelay(GIMBAL_TASK_INIT_TIME);								  // ���ܵĻ���GIMBAL_TASK_INIT_TIMEʱ��Ķ�
	gimbal_init(&gimbal_pid, &cali_gimbal_pid, &gimbal_control_data); 

	while (1)
	{
		current_time = xTaskGetTickCount();												   //��ǰϵͳʱ��       *hyj
		send_gyro_data_to_chassis();													   ///��ȡGYRO���������ݣ�ͨ��can2 mail1����������
		gimbal_work_mode_update(&rc_ctrl_data, &gimbal_control_data);					   //����ң�����ݸ�����̨״̬
		gimbal_set_and_fdb_update(&gimbal_control_data, robot_control_mode, control_data); //set fdb���ݸ���  
		gimbal_cascade_pid_calculate(&gimbal_pid, &gimbal_control_data);				   ///����pid���� 
		gimbal_control_loop(&gimbal_pid, &gimbal_control_data);							   ///����̨�������ָ�������̨�˶�
		set_GUI_task_state();															   //����GUI����״̬

		vTaskDelayUntil(&current_time, GIMBAL_TASK_TIME_1MS); //1msһ��         *hyj
	}
}
