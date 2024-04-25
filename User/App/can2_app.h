#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

//#define RATE_BUF_SIZE 6  //����ٶȻ����˲����ڴ�С

typedef struct
{
	
	uint8_t robot_id;                           //������ID
	//�������
	uint16_t shooter_id1_42mm_cooling_rate;     //ÿ����ȴֵ0��17mm��  1��42mm��  
	uint16_t shooter_id1_42mm_cooling_limit;    //ǹ����������
	uint16_t shooter_id1_42mm_speed_limit;      //�ٶ�����
	uint8_t mains_power_shooter_output ;        //��Դ�Ƿ�������������
	float bullet_speed;                         //��ǰ����
	uint16_t shooter_id1_42mm_cooling_heat;     //17mm��ǰǹ������
	uint8_t hurt_type : 4;                      //0x2 �����ٿ�Ѫ��0x3 ��ǹ��������Ѫ��

} ext_Judge_data_t;                     //ʵʱ������Ϣ   *hyj

typedef enum
{
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x1FE,	// ң������ch2,ch3����

	CAN2_CONNECT_CHECK_STD_ID = 0x203,

	CAN2_GIMBAL_STD_ID = 0x1FF,
	CAN2_YAW_MOTOR_STD_ID = 0x205,	// yaw���1
	CAN2_YAW_MOTOR2_STD_ID = 0x206,	// yaw���2
	CAN2_PITCH_MOTOR_STD_ID = 0x207,	//pitch���
	CAN2_PITCH1_MOTOR_STD_ID = 0x208,	//pitch��� 
	
	CAN2_CONNECT_CM_GYRO_STD_ID = 0x209,
 
	CAN2_CHASSIS_PID_ROTATE_STD_ID = 0X20A,
	CAN2_CHASSIS_PID_CM_STD_ID = 0X20B,	
	CAN2_SHOOT_42mm_ID = 0x020C,         //17mm�������������Ϣ
	CAN2_SHOOT_JUDGE_ID = 0x020D,        //�������������Ϣ
	CAN2_CONNECT_CM_SPEED_STD_ID = 0x20E,	// ���̵���ٶ�
	
} can2_msg_id_e;

extern motor_msg_t yaw_motor_msg;
extern motor_msg_t pitch_motor_msg;
extern motor_msg_t pitch1_motor_msg ;

void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
                             
void shoot_42mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[]);
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[]);

// void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq);
/************************************2��pitch***********************************/
void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq1,int16_t pitch_iq2);
void set_gimbal_stop(void);

motor_msg_t *get_yaw_motor_msg_point(void);
motor_msg_t *get_pitch_motor_msg_point(void);
motor_msg_t *get_pitch1_motor_msg_point(void);
#endif

