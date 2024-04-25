/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description: 
 * @Note:       1.云台电机初始化 可以考虑不使用串级pid这样可以减少振荡 更快到达目标值
				2.目前是计算云台初始化角度作为控制的初始角度，减少mpu复位这一部分
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

/*二阶卡尔曼*/
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
speed_calc_data_t Vision_Yaw_speed_Struct; //预测速度结构体
speed_calc_data_t Vision_Pitch_speed_Struct;
kalman_filter_t yaw_kalman_filter; //卡尔曼结构体
kalman_filter_t pitch_kalman_filter;

kalman_filter_init_t yaw_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {200, 0, 0, 400} //500 1000
};							   //初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {200, 0, 0, 400}};						//初始化pitch的部分kalman参数
float constrain_float(float amt, float low, float high) //限制函数
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
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
void robot_control_mode_update(RC_ctrl_t *rc_s) ///遥控器控制机器人模式
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
	// if(monitor.exist_error_flag == 1)//发生严重错误时，不受遥控指挥强行转为调试模式，这里如果不注释掉，一旦有电机掉线就无法动
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
		set_robot_work_mode(ROBOT_INIT_MODE); //初始化
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
	case GIMBAL_RELATIVE_ANGLE_MODE: //相对角度，使用编码值
	{
		gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
		gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
	}
	break;
	case GIMBAL_ABSOLUTE_ANGLE_MODE: //绝对角度，使用陀螺仪
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
float speed_threshold = 5.f; //速度过快
float debug_speed;			 //左正右负,一般都在1左右,debug看

float pitch_vision_Kp = 0.5;//0.5
float yaw_vision_Kp = 1;

float pitch_vision_Kd = 0.5;
float yaw_vision_Kd = 0.5;
/**
  * @brief      视觉同位置补偿
  * @author         
  * @param[in]      
  * @retval			
  * @note        补偿完要减去补偿前测的偏差角度
  */
float yaw_install_k = 0;
float pitch_install_k = 0;
void vision_install_compensate(void)
{
	control_data.yaw_compensate = (atan(yaw_install_Difference / control_data.Target_distance) * yaw_install_k);
	// if( control_data.pitch_dev > 0 )	//需要往上抬头，value要变大
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
  * @brief      视觉补偿
  * @author         
  * @param[in]      
  * @retval			
  * @note        yaw通过陀螺仪的变化预测位置  pitch通过距离变换高度
  */

float yaw_INS_com_k =0; //0.40; 不加比较好
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
	if(((yaw_INS_err)>1.0f)||((yaw_INS_err)<-1.0f))//较小的变化不要
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
  * @brief      重力补偿
  * @author         
  * @param[in]      
  * @retval			
  * @note        对面差值（分子），可改      *hyj
  */
float vision_G = 54;          //不能太大
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
     //打地面
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
//    // 带入求根公式，解出t^2
//    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
////    std::cout << fmt::format("a:{}, b:{}, c:{}", a, b, c) << std::endl;
////    std::cout << "t2:" << t_2 << std::endl;
//    double fly_time = sqrt(t_2);                                       // 子弹飞行时间（单位:s）
//    // 解出抬枪高度，即子弹下坠高度
//    double height = 0.5 * 9.8 * t_2;
//     control_data.Gravity_compensation=height+54;
SortAver_Filter(control_data.Gravity_compensation,&GG,25);  //去极值平均滤波
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
//弹道下坠
float dropOffset(float distance, float shootSpd) //下坠补偿。
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
  * @brief        目标速度计算
  * @author         
  * @param[in]    预测目标的速度、当前时间、预测目标时枪管
  * @retval		  预测的目标速度
  * @note  		
  */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;
	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2; //计算速度
		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}
	if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0; //时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed; //计算出的速度
}
/***************自瞄******************/
//误差
float Auto_Error_Yaw[2]; //    now/last
float Auto_Error_Pitch[2];
float Auto_Distance; //距离单目
//自瞄突然开启,卡尔曼滤波开启延时
uint16_t Auto_KF_Delay = 0;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch; //卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;					//二阶卡尔曼滤波结果,0角度 1速度
vision_KM_debug_data_t vision_KM;
/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 没有加I会有误差,只在系统启动时调用一次
  */
void GIMBAL_InitArgument(void)
{
/*--------------自瞄角度补偿初始化----------------*/
#if INFANTRY_DEBUG_ID == DEBUG_ID_ONE

	debug_y_sk = 88; //38;//35;//30;//移动预测系数,越大预测越多
	debug_y_sb_sk = 10;
	debug_p_sk = 26;	   //移动预测系数,越大预测越多
	debug_auto_err_y = 16; //15;//10;//15;//角度过大关闭预测
	debug_auto_err_p = 8;
	debug_kf_delay = 180;		//100;//200;//120;//150;//预测延时开启
	debug_kf_speed_yl = 0.38;	//0.1;//0.2;//0.1;//0.08;//0.1;//0.3;//速度过低关闭预测
	debug_kf_speed_yh = 6;		//速度过高关闭预测
	debug_kf_speed_yl_sb = 0.1; //
	debug_kf_speed_pl = 0.1;	//pitch速度过低关闭预测
	debug_kf_y_angcon = 260;	//125;//115;//135;//预测量限幅
	debug_kf_p_angcon = 45;		//pitch预测量限幅

	//打符
	//底盘
	debug_y_mid = Mech_Mid_Yaw;
	debug_p_mid = Mech_Mid_Pitch;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//云台
	Buff_Pitch_Comp_Gimbal = 0;
	Buff_Yaw_Comp_Gimbal = 0;

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 1;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = 0;

#elif INFANTRY_DEBUG_ID == DEBUG_ID_TWO

	debug_y_sk = 52;		 //45;//14.8;//移动预测系数,越大预测越多
	debug_y_sb_sk = 62;		 //61;//55;
	debug_y_sb_brig_sk = 88; //
	debug_p_sk = 20;		 //移动预测系数,越大预测越多
	debug_auto_err_y = 120;	 //角度过大关闭预测
	debug_auto_err_p = 150;
	debug_kf_delay = 80;		//预测延时开启
	debug_kf_speed_yl = 0.2;	//0.35;//速度过低关闭预测
	debug_kf_speed_yl_sb = 0.2; //
	debug_kf_speed_yh = 4.2;	//速度过高关闭预测
	debug_kf_speed_pl = 0.15;	//pitch速度过低关闭预测
	debug_kf_y_angcon = 220;	//125;//115;//135;//预测量限幅
	debug_kf_p_angcon = 45;		//pitch预测量限幅

	//打符
	//底盘
	debug_y_mid = 4122; //5883;
	debug_p_mid = 3860; //3565;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//云台
	//7.1米
	Buff_Pitch_Comp_Gimbal = -78; //-88;//家里-88，单项赛-96
	//8米
	//Buff_Pitch_Comp_Gimbal = -86;//-96;
	Buff_Yaw_Comp_Gimbal = -8; //28m/s

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 0.99;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = -20;

#elif INFANTRY_DEBUG_ID == DEBUG_ID_THREE

	debug_y_sk = 45; //14.8;//移动预测系数,越大预测越多
	debug_y_sb_sk = 55;
	debug_y_sb_brig_sk = 90; //
	debug_p_sk = 20;		 //移动预测系数,越大预测越多
	debug_auto_err_y = 120;	 //角度过大关闭预测
	debug_auto_err_p = 150;
	debug_kf_delay = 80;		//预测延时开启
	debug_kf_speed_yl = 0.2;	//速度过低关闭预测
	debug_kf_speed_yl_sb = 0.2; //
	debug_kf_speed_yh = 4;		//速度过高关闭预测
	debug_kf_speed_pl = 0.15;	//pitch速度过低关闭预测
	debug_kf_y_angcon = 220;	//125;//115;//135;//预测量限幅
	debug_kf_p_angcon = 45;		//pitch预测量限幅

	//打符
	//底盘
	debug_y_mid = Mech_Mid_Yaw;
	debug_p_mid = Mech_Mid_Pitch;
	Buff_Pitch_Comp = 0;
	Buff_Yaw_Comp = 0;

	//云台
	//7.1米
	Buff_Pitch_Comp_Gimbal = -22; //-30;//-24;//-20;//-10;//家里-24，单项赛-30
	//8米
	//Buff_Pitch_Comp_Gimbal = -30;

	Buff_Yaw_Comp_Gimbal = -13; //19;

	Buff_Pitch_Correct_Chassis = 1;
	Buff_Yaw_Correct_Chassis = 1;
	Buff_Pitch_Correct_Gimbal = 1;
	Buff_Yaw_Correct_Gimbal = 1;

	Base_Yaw_Comp_Gimbal = -13;
#elif INFANTRY_DEBUG_ID == DEBUG_ID_FOUR

	vision_KM.debug_y_sk = 50;		   //45;//35;//14.8;//yaw移动预测系数,越大预测越多
	vision_KM.debug_y_sb_sk = 59;	   //55;         //哨兵用
	vision_KM.debug_y_sb_brig_sk = 90; //       //哨兵用
	vision_KM.debug_p_sk = 20;		   //pitch移动预测系数,越大预测越多

	vision_KM.debug_auto_err_y = 120; //yaw角度过大时关闭预测
	vision_KM.debug_auto_err_p = 150; //pitch角度过大时关闭预测

	vision_KM.debug_kf_delay = 80;		  //预测延时开启
	vision_KM.debug_kf_speed_yl = 0.2;	  //0.35;  //yaw速度过低关闭预测
	vision_KM.debug_kf_speed_yl_sb = 0.2; //0.2;//哨兵用
	vision_KM.debug_kf_speed_yh = 5;	  //yaw速度过高关闭预测
	vision_KM.debug_kf_speed_pl = 0.15;	  //pitch速度过低关闭预测
	vision_KM.debug_kf_y_angcon = 220;	  //125;//115;//135;//yaw预测量限幅
	vision_KM.debug_kf_p_angcon = 45;	  //pitch预测量限幅
#endif

	//卡尔曼滤波器初始化

	/*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q, 2, 2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R, 2, 2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

	mat_init(&pitch_kalman_filter.Q, 2, 2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R, 2, 2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
}
/**********************************************************************************/
/** 
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */
uint8_t view_control_flag = 0;
float debug_y_dk = 450;	  //yaw距离预测比例，越大预测越少
uint32_t Vision_Time[2];  // NOW/LAST
int vision_time_js;		  //计算视觉延迟
float error_yaw_k = 1;	  //7.5;//5.6;//2.2;//误差放大
float error_pitch_k = 10; //5;//3;//2.1;//误差放大
float debug_kf_y_angle;	  //yaw预测暂存
float debug_kf_p_angle;	  //pitch预测暂存
//根据距离调节预测比例和限幅
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;
float pitch_speed_k = 0;
float kf_pitch_angcon = 0;
float debug_kf_angle_temp;		//预测角度斜坡暂存量
float debug_kf_angle_ramp = 20; //预测角度斜坡变化量
float debug_kf_dist;
float debug_dist_bc = 0;
float gim_auto_ramp_y = 5; //10;//刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
float gim_auto_ramp_p = 5; //刚开启自瞄时缓慢移过去，防止视觉拖影掉帧
float kf_speed_yl = 0;	   //
void GIMBAL_AUTO_Mode_Ctrl(gimbal_control_data_t *gimbal_control_data)
{
	static float yaw_angle_raw, pitch_angle_raw; //卡尔曼滤波角度测量值
	static float yaw_angle_ref;					 //记录目标角度
	static float pitch_angle_ref;				 //记录目标角度
	float kf_delay_open = 0;
	kf_speed_yl = vision_KM.debug_kf_speed_yl; //yaw速度过低关闭预测

	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if (view_control_flag == 1) //视觉数据更新了
	{
		//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref = (gimbal_control_data->gimbal_INS->yaw_angle - (control_data.yaw_dev));
		pitch_angle_ref = (gimbal_control_data->gimbal_INS->roll_angle + (control_data.pitch_dev * 0.2f));
		view_control_flag = 0;					//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount(); //获取新数据到来时的时间
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if (Vision_Time[NOW] != Vision_Time[LAST]) //更新新数据到来的时间
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];
		//更新二阶卡尔曼滤波测量值
		yaw_angle_raw = yaw_angle_ref;
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	//目标速度解算
	if (control_data.frame_seq == 4) //识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);
	}
	else
	{
		//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), gimbal_control_data->gimbal_INS->yaw_angle);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), gimbal_control_data->gimbal_INS->roll_angle);
		//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, gimbal_control_data->gimbal_INS->yaw_angle, 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, gimbal_control_data->gimbal_INS->roll_angle, 0);
	}
	//未识别到目标时鼠标可随意控制云台
	if (control_data.frame_seq == 4) //识别到了目标     //未识别到怎么还是4？   *hyj
	{
		Auto_KF_Delay++; //滤波延时开启
						 //		if(VisionRecvData.auto_too_close == TRUE
						 //			&& (Chassis_IfCORGI() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE) )//目标距离太近，减小预测
						 //		{
						 //			yaw_speed_k = vision_KM.debug_y_sk;///4.f;//3.f;//预测系数减半
						 //			kf_yaw_angcon = vision_KM.debug_kf_y_angcon;//3.f;//2.f;
						 //			kf_speed_yl = vision_KM.debug_kf_speed_yl;
						 //		}
						 //		else//正常预测量
						 //		{
						 //			if( GIMBAL_AUTO_PITCH_SB() == TRUE )  //根据角度判断是否在打哨兵 下一代的人开发的时候自己整吧
						 //			{
						 //				yaw_speed_k = vision_KM.debug_y_sb_sk;
						 //				kf_yaw_angcon = vision_KM.debug_kf_y_angcon;
						 //				kf_speed_yl = vision_KM.debug_kf_speed_yl_sb;
						 //
						 //				if(IF_KEY_PRESSED_G)  //或者到时操作手来控制 我们现在就靠操作手打哨兵了
						 //				{
						 //					yaw_speed_k = vision_KM.debug_y_sb_brig_sk;
						 //					kf_yaw_angcon = vision_KM.debug_kf_y_angcon*1.1f;
						 //					kf_speed_yl = vision_KM.debug_kf_speed_yl_sb*0.4f;//0.9f;
						 //				}
						 //			}
		if (1)			 //一直用这个参数吧
		{
			yaw_speed_k = vision_KM.debug_y_sk;
			kf_yaw_angcon = vision_KM.debug_kf_y_angcon;
			kf_speed_yl = vision_KM.debug_kf_speed_yl;
		}
		//		}
		/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑二阶卡尔曼计算↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

		/*预测开启条件*/
		if (fabs(Auto_Error_Yaw[NOW]) < vision_KM.debug_auto_err_y																							 //debug看
			&& Auto_KF_Delay > kf_delay_open && fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl && fabs(yaw_kf_result[KF_SPEED]) < vision_KM.debug_kf_speed_yh) //在yaw角度不大 速度不低也不快时可以开预测
		{
			if (yaw_kf_result[KF_SPEED] >= 0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1; //debug_kf_dist;
			}
			else if (yaw_kf_result[KF_SPEED] < 0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1; //debug_kf_dist;
			}
			//			debug_kf_angle_temp = vision_KM.debug_y_sk * yaw_kf_result[KF_SPEED];//此处不需要速度太慢关预测
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -vision_KM.debug_kf_y_angcon, vision_KM.debug_kf_y_angcon); //预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);							   //使预测量缓慢变化
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -vision_KM.debug_kf_y_angcon, vision_KM.debug_kf_y_angcon);

			gimbal_control_data->gimbal_yaw_set = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle; //vision_KM.debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
		}
		/*预测条件没达到，关闭预测*/
		else
		{
			gimbal_control_data->gimbal_yaw_set = yaw_angle_ref;
			//			if( fabs(Auto_Error_Yaw[NOW]) < 1.5f )//接近目标
			//			{
			//				mobpre_yaw_stop_delay++;
			//				if(mobpre_yaw_stop_delay > 25)//停止稳定50ms
			//				{
			//
			//				}
			//			}
		}
		/*---------------pitch给个很小的预测------------------*/
		if (Auto_KF_Delay > vision_KM.debug_kf_delay && fabs(Auto_Error_Pitch[NOW]) < vision_KM.debug_auto_err_p && fabs(pitch_kf_result[KF_SPEED]) > vision_KM.debug_kf_speed_pl
			// && (GIMBAL_AUTO_PITCH_SB_SK() == FALSE || GIMBAL_AUTO_PITCH_SB() == FALSE) //是否中短距离瞄哨兵    //*hyj    先注释
			// 	&& VisionRecvData.distance/100 < 4.4f                                                            //等距离     先注释
		)
		{
			if (1) //(VisionRecvData.auto_too_close == TRUE)//目标距离太近，减小预测                                                  //先注释
			{
				pitch_speed_k = vision_KM.debug_p_sk / 2.f; //预测系数减半
				kf_pitch_angcon = vision_KM.debug_kf_p_angcon / 1.5f;
			}
			else //正常预测量
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
			//pitch预测量限幅
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);

			gimbal_control_data->gimbal_pitch1_set = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle; //一个pitch:gimbal_control_data->gimbal_pitch_set
		}
		/*预测条件没达到，关闭预测*/
		else
		{
			gimbal_control_data->gimbal_pitch1_set = pitch_angle_ref; //一个pitch:gimbal_control_data->gimbal_pitch_set
		}
	} 
	else //未识别到目标,可随意控制云台 留给你的小礼物
	{
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, control_data.yaw_dev, 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, control_data.pitch_dev, 0);
	}
}

/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval 
  * @attention	里面是视觉模式中，一旦视觉标志丢失就转为操作模式   双pitch  II
  */
void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,
							   robot_control_mode_e robot_control_mode,
							   _tx2_control_data control_data)
{
	if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO) //mpu
	{
		if (robot_control_mode == KEY_MOUSE_MODE) //KEY_MOUSE_MODE
		{
			if (shoot_control_data.shoot_vision_flag && view_control_flag) //按F键视觉自瞄     *hyj   加上卡尔曼滤波
			{
				gimbal_control_data->gimbal_yaw_set = (gimbal_control_data->gimbal_INS->yaw_angle + control_data.yaw_compensate - (control_data.yaw_dev)); // gimbal_control_data->gimbal_pitch_set = (gimbal_control_data->gimbal_INS->roll_angle + (control_data.pitch_dev*0.2f));
				// GIMBAL_AUTO_Mode_Ctrl(gimbal_control_data);
				// gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y, \
				// 				-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
				// 				MOUSE_SINGLE_TIME_ALLOW_VALUE);
				// gimbal_control_data->gimbal_pitch_set += (gimbal_control_data->rc_ctrl->mouse.y) * \
				// 										gimbal_control_data->rc_ctrl->pitch_sensit;
				view_control_flag = 0; //清空视觉数据标志位
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
		// else if(robot_control_mode != KEY_MOUSE_MODE) //REMOTE_MODE		调试视觉用
		else if (robot_control_mode == REMOTE_MODE) //REMOTE_MODE
		{
			
			//视觉处理     *hyj
			if (view_control_flag == 1)
			{
				vision_install_compensate();//视觉同位置补偿
				// vision_Gravity_compensate();
				vision_INS_distance_compensation(&control_data,gimbal_control_data);
				
				gimbal_control_data->gimbal_yaw_set = (
																		-gimbal_control_data->gimbal_INS->yaw_angle +  //当前姿态角
																			control_data.yaw_compensate + 								//视觉同位置补偿
																		(control_data.yaw_dev*yaw_vision_Kp)+					//位置
																		constrain_float(yaw_vision_Kd*control_data.yaw_dev_err,-1,1)+	//位置误差
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
															  GAMBAL_PITCH_MAX_ANGLE_SET_FACT ; //ch1 双Pitch 调遥控控制灵敏度
		 		gimbal_val_limit_float((&gimbal_control_data->gimbal_pitch1_set1),
								 		-45.0, 
								 		45.0);//角度限幅 
				gimbal_control_data->gimbal_pitch1_set = gimbal_control_data->gimbal_pitch1_set1+90;
				

				gimbal_control_data->gimbal_pitch2_set1 += (gimbal_control_data->rc_ctrl->rc.ch1) *
															  GAMBAL_PITCH_MAX_ANGLE_SET_FACT; //ch1 双Pitch
				gimbal_val_limit_float((&gimbal_control_data->gimbal_pitch2_set1),
								 		-45.0,
								 		45.0);
				gimbal_control_data->gimbal_pitch2_set = gimbal_control_data->gimbal_pitch2_set1+90;
			}
		}
		gimbal_control_data->gimbal_yaw_fdb = -(gimbal_control_data->gimbal_INS->yaw_angle);

		// gimbal_control_data->gimbal_pitch1_fdb = gimbal_control_data->gimbal_pitch_motor_msg->encoder.ecd_angle-2;
 		// gimbal_control_data->gimbal_pitch2_fdb = gimbal_control_data->gimbal_pitch1_motor_msg->encoder.ecd_angle;//编码器

		gimbal_control_data->gimbal_pitch1_fdb = -( gimbal_control_data->gimbal_INS->roll_angle); //2 pitch
		gimbal_control_data->gimbal_pitch2_fdb = -( gimbal_control_data->gimbal_INS->roll_angle); //2 pitch //陀螺仪
	}

	else if (gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_ENCONDE) //初始化时的编码模式 set 可修改
	{
		// 比赛底盘找云台
		// gimbal_control_data->gimbal_yaw_set = -gimbal_control_data->gimbal_INS->yaw_angle;
		// 调试时云台找底盘，安全第一
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
	//Pitch限幅
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
/*******************************2个pitch   II**************************************/
void gimbal_cascade_pid_calculate(gimbal_pid_t *gimbal_pid,
								  gimbal_control_data_t *gimbal_control_data)
{
	gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.fdb = gimbal_control_data->gimbal_yaw_fdb;
	gimbal_pid->yaw_pid.position_pid.Calc(&gimbal_pid->yaw_pid.position_pid);

	gimbal_pid->yaw_pid.speed_pid.set = gimbal_pid->yaw_pid.position_pid.output; ///把外环位置输出作为内环速度输入
	if (gimbal_control_data->yaw_motor_fdb_mode == GIMBAL_MOTOR_GYRO || 1)
	{
		gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y; ///陀螺仪原始数据作为内环速度输入
	}
	else
	{
		gimbal_pid->yaw_pid.speed_pid.fdb = 0;
	}
	gimbal_pid->yaw_pid.speed_pid.Calc(&gimbal_pid->yaw_pid.speed_pid);

	//pitch1
	gimbal_pid->pitch1_pid.position_pid.set = gimbal_control_data->gimbal_pitch1_set; //期望值给个固定值，赋值在上面的set和fdb更新那里
	gimbal_pid->pitch1_pid.position_pid.fdb = gimbal_control_data->gimbal_pitch1_fdb; //等于陀螺仪的角度值；同时运行两个电机的话连接两个电机的杆会不会断？）
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
/********************************2个pitch  II***************************************/
void gimbal_control_loop(gimbal_pid_t *gimbal_pid,
						 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->given_current.yaw_motor = -gimbal_pid->yaw_pid.speed_pid.output; //g6020反向安装需要加负号
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
	//yaw cascade pid ///校准后的yaw角度和速度赋给云台
	gimbal_pid_init(&gimbal_pid->yaw_pid.position_pid, &cali_pid->yaw_pid.position); //后面赋给前面
	gimbal_pid_init(&gimbal_pid->yaw_pid.speed_pid, &cali_pid->yaw_pid.speed);
	//pitch cascade pid
	/*******************两个pitch*******************************/
	gimbal_pid_init(&gimbal_pid->pitch1_pid.position_pid, &cali_pid->pitch1_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch1_pid.speed_pid, &cali_pid->pitch1_pid.speed);
	gimbal_pid_init(&gimbal_pid->pitch2_pid.position_pid, &cali_pid->pitch2_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch2_pid.speed_pid, &cali_pid->pitch2_pid.speed);

	// set_robot_control_mode(GUI_CALI_MODE);
	// set_robot_work_mode(ROBOT_CALI_MODE);
	set_gimbal_work_mode(GIMBAL_CALI_MODE);
	/*设置刚上电后机器人的工作模式，主要用于按键鼠标模式的上电*/
	set_robot_control_mode(REMOTE_MODE);
	set_robot_work_mode(ROBOT_COMMON_MODE); //普通底盘跟随模式
}
/**
  * @brief        运行时挂起GUI任务
  * @author         
  * @param[in]      
  * @retval			
  * @note         ///校准模式下获取遥控器模拟键值  
  */

extern TaskHandle_t GUI_Task_Handler;
void set_GUI_task_state(void)
{
	if (get_robot_control_mode() == GUI_CALI_MODE)
	{
		if (eTaskGetState(GUI_Task_Handler) == eSuspended)
		{
			vTaskResume(GUI_Task_Handler);	   //恢复任务
			HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn); //开启定时器获取rc模拟健值
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

		REDLED_ON(); ///红灯标志位
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
  * @retval while循环的前面两个函数消耗4%(其中第一个0.3%) 整个while消耗15%
  */
void gimbal_task(void *argument)
{
	TickType_t current_time = 0;
	vTaskDelay(GIMBAL_TASK_INIT_TIME);								  // 可能的话把GIMBAL_TASK_INIT_TIME时间改短
	gimbal_init(&gimbal_pid, &cali_gimbal_pid, &gimbal_control_data); 

	while (1)
	{
		current_time = xTaskGetTickCount();												   //当前系统时间       *hyj
		send_gyro_data_to_chassis();													   ///获取GYRO陀螺仪数据，通过can2 mail1发送往底盘
		gimbal_work_mode_update(&rc_ctrl_data, &gimbal_control_data);					   //根据遥控数据更新云台状态
		gimbal_set_and_fdb_update(&gimbal_control_data, robot_control_mode, control_data); //set fdb数据更新  
		gimbal_cascade_pid_calculate(&gimbal_pid, &gimbal_control_data);				   ///串级pid计算 
		gimbal_control_loop(&gimbal_pid, &gimbal_control_data);							   ///给云台电调发送指令控制云台运动
		set_GUI_task_state();															   //设置GUI任务状态

		vTaskDelayUntil(&current_time, GIMBAL_TASK_TIME_1MS); //1ms一次         *hyj
	}
}
