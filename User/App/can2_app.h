#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

//#define RATE_BUF_SIZE 6  //电机速度滑动滤波窗口大小

typedef struct
{
	
	uint8_t robot_id;                           //机器人ID
	//发射机构
	uint16_t shooter_id1_42mm_cooling_rate;     //每秒冷却值0：17mm的  1：42mm的  
	uint16_t shooter_id1_42mm_cooling_limit;    //枪口热量上限
	uint16_t shooter_id1_42mm_speed_limit;      //速度上限
	uint8_t mains_power_shooter_output ;        //电源是否输出给发射机构
	float bullet_speed;                         //当前射速
	uint16_t shooter_id1_42mm_cooling_heat;     //17mm当前枪口热量
	uint8_t hurt_type : 4;                      //0x2 超射速扣血；0x3 超枪口热量扣血；

} ext_Judge_data_t;                     //实时裁判信息   *hyj

typedef enum
{
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x1FE,	// 遥控数据ch2,ch3发送

	CAN2_CONNECT_CHECK_STD_ID = 0x203,

	CAN2_GIMBAL_STD_ID = 0x1FF,
	CAN2_YAW_MOTOR_STD_ID = 0x205,	// yaw电机1
	CAN2_YAW_MOTOR2_STD_ID = 0x206,	// yaw电机2
	CAN2_PITCH_MOTOR_STD_ID = 0x207,	//pitch电机
	CAN2_PITCH1_MOTOR_STD_ID = 0x208,	//pitch电机 
	
	CAN2_CONNECT_CM_GYRO_STD_ID = 0x209,
 
	CAN2_CHASSIS_PID_ROTATE_STD_ID = 0X20A,
	CAN2_CHASSIS_PID_CM_STD_ID = 0X20B,	
	CAN2_SHOOT_42mm_ID = 0x020C,         //17mm发射机构裁判信息
	CAN2_SHOOT_JUDGE_ID = 0x020D,        //发射机构裁判信息
	CAN2_CONNECT_CM_SPEED_STD_ID = 0x20E,	// 底盘电机速度
	
} can2_msg_id_e;

extern motor_msg_t yaw_motor_msg;
extern motor_msg_t pitch_motor_msg;
extern motor_msg_t pitch1_motor_msg ;

void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
                             
void shoot_42mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[]);
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[]);

// void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq);
/************************************2个pitch***********************************/
void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq1,int16_t pitch_iq2);
void set_gimbal_stop(void);

motor_msg_t *get_yaw_motor_msg_point(void);
motor_msg_t *get_pitch_motor_msg_point(void);
motor_msg_t *get_pitch1_motor_msg_point(void);
#endif

