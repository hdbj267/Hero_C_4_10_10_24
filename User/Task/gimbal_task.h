#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "remote_app.h"
#include "INS_task.h"



#define GIMBAL_TASK_TIME_1MS                  (1u)           

#define ROBOT_COMMON_MODE_KEY                 KEY_PRESSED_OFFSET_CTRL            
#define ROBOT_ROTATE_STOP_MODE_KEY            KEY_PRESSED_OFFSET_E        
#define ROBOT_ROTATE_MOTION_MODE_KEY          KEY_PRESSED_OFFSET_Q 

#define GIMBAL_TASK_INIT_TIME                 (3300)

#define GAMBAL_YAW_MAX_ANGLE_SET_FACT         (0.0002f)   // (0.0002f)   //遥控遥杆灵敏度0.0005
#define GAMBAL_PITCH_MAX_ANGLE_SET_FACT       (0.0001f)
           
#define MOUSE_SINGLE_TIME_ALLOW_VALUE  38   //150

#define GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT (-0.005f)    //大位移鼠标灵敏度
#define GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT   (-0.005f)
#define GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT (-0.008f)    //小位移鼠标灵敏度
#define GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT   (-0.008f)

#define GAMBAL_YAW_INIT_ENCODE_VALUE_OU     (3859)            
#define GAMBAL_YAW_INIT_ENCODE_VALUE_JI      (1832)               
#define GAMBAL_PITCH_INIT_ENCODE_VALUE        (4015.0f)  //4093.0f

#define GAMBAL_YAW_angle_VALUE     (22.755555f)                    //yaw  编码转为角度
#define GAMBAL_PITCH_angle_VALUE   (22.755555f)                    //pitch
#define CAMBAL_PITHC_LEVEL_ANGLE_TO_ENCODE    (3700.0f)//云台水平值
#define CAMBAL_PITCH_MIN_ANGLE_TO_ENCODE      (3200.0f)
#define CAMBAL_PITCH_MAX_ANGLE_TO_ENCODE      (4285.0f)//低头下限

#define YAW_FOLLOW_clipped			(1.5f)		//自瞄模式下 吊射时Yaw的跟随值（单项赛用）
#define PITCH_FOLLOW_clipped		(66.5f)		//自瞄模式下 吊射时Pitch的跟随值

#define YAW_FOLLOW_CLOSE			(-14.35f)		//自瞄模式下 近距离时Yaw的跟随值（3 v 3用）
#define PITCH_FOLLOW_CLOSE			(-92.0f)	//自瞄模式下 近距离时Pitch的跟随值  -98.0  vision_k=1;-96.0;position_p=10;// visiom_k=0.5;-104.position_p=10

#define GAMBAL_ENCODE_TO_ANGLE                ((float)0.0439506775729459)  //   360.0/8191.0

#define	yaw_install_Difference					(35.0f)
#define	pitch_install_Difference				(40.0f)

/* 步兵ID预编译,仅适用于调试,分区赛国赛需另外对应 */
#define    DEBUG_ID_ONE     1		//旧步兵
#define    DEBUG_ID_TWO     2		//新步兵
#define    DEBUG_ID_THREE   3		//英雄
#define    DEBUG_ID_FOUR    4		//哨兵

#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

#define NOW  0
#define LAST 1

#define    INFANTRY_DEBUG_ID    DEBUG_ID_FOUR
typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
  
}speed_calc_data_t;

typedef enum
{
	ROBOT_CALI_MODE = 0,     //调试模式
	ROBOT_INIT_MODE,	     //初始化
	ROBOT_INIT_END_MODE,     //初始化结束切换点
	ROBOT_COMMON_MODE,       //普通底盘跟随模式
	ROBOT_ROTATE_STOP_MODE,  //静止小陀螺
	ROBOT_ROTATE_MOTION_MODE,//运动小陀螺
	ROBOT_VISION_MODE,       //视觉自瞄模式
	ROBOT_ERROR_MODE,        //错误
}robot_work_mode_e;

typedef enum
{
	GIMBAL_CALI_MODE = 0,
	GIMBAL_INIT_MODE,  
	GIMBAL_ABSOLUTE_ANGLE_MODE,
	GIMBAL_RELATIVE_ANGLE_MODE, 
//	GIMBAL_MOTIONLESS, 	
}gimbal_work_mode_e;
  
typedef enum
{
    GIMBAL_MOTOR_RAW = 0,    //电机原始值控制
    GIMBAL_MOTOR_GYRO,       //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE,    //电机编码值角度控制
}gimbal_motor_feedback_mode_e;

typedef enum
{
	KEY_MOUSE_MODE = 0,
	REMOTE_MODE,
	GUI_CALI_MODE,
}robot_control_mode_e;




typedef struct
{
	int16_t yaw_motor;
	int16_t pitch1_motor;
	int16_t pitch2_motor;
	int16_t trigger_motor;
	int16_t fric1_motor;
	int16_t fric2_motor;
}given_current_t;


typedef struct
{
	cascade_pid_t yaw_pid;	
	cascade_pid_t pitch1_pid;
	cascade_pid_t pitch2_pid;
}gimbal_pid_t;


typedef struct
{
	RC_ctrl_t *rc_ctrl;
	const INS_t *gimbal_INS;
	motor_msg_t *gimbal_yaw_motor_msg;
	motor_msg_t *gimbal_pitch_motor_msg;
	motor_msg_t *gimbal_pitch1_motor_msg;
	
	given_current_t given_current;
	
	gimbal_motor_feedback_mode_e yaw_motor_fdb_mode;
	gimbal_motor_feedback_mode_e pitch_motor_fdb_mode;
	
	float gimbal_yaw_set;
	float gimbal_yaw_fdb;
	float gimbal_pitch1_set;
	float gimbal_pitch1_set1;
	float gimbal_pitch1_fdb;
	float gimbal_pitch2_set1;
	float gimbal_pitch2_set;
	float gimbal_pitch2_fdb;

	int16_t vision_yaw;
	int16_t vision_pitch;
	
}gimbal_control_data_t;

typedef struct
{
	float debug_y_sk;// = 38;//35;//30;//yaw移动预测系数,越大预测越多
	float debug_y_sb_sk;//哨兵预测系数
	float debug_y_sb_brig_sk;//桥头哨兵
	float debug_p_sk;//pitch移动预测系数,越大预测越多

	float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
	float debug_auto_err_p;//pitch角度过大关闭预测
	float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
	float debug_kf_speed_yl;//yaw速度过低关闭预测
	float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
	float debug_kf_speed_yh;//yaw速度过高关闭预测
	float debug_kf_speed_pl;//pitch速度过低关闭预测
	float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
	float debug_kf_p_angcon;//pitch预测量限幅

	float debug_kf_speed_ph;

}vision_KM_debug_data_t;


extern given_current_t given_current;
uint8_t get_robot_control_mode(void);
uint8_t get_robot_work_mode(void);
uint8_t get_gimbal_work_mode(void);
void set_robot_control_mode(robot_control_mode_e mode);

void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,   \
							   robot_control_mode_e robot_control_mode, 
							   _tx2_control_data control_data);
#endif
