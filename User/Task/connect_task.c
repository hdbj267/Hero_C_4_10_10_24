/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description:    1.开机云台发送效验数据包，确定底盘和云台是否成功连接，
					  未连接则挂起任务调度器。		
 * @Others: 
**/
#include "connect_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "remote_app.h"
#include "can1_app.h"
#include "gimbal_task.h"
#include "can.h"
#include "can2_app.h"
#include "tim.h"
#include "GUI_task.h"
#include "shoot_task.h"
#include "usart.h"


extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;

extern gimbal_control_data_t gimbal_control_data;
extern shoot_control_data_t shoot_control_data;
extern ext_Judge_data_t Judge_data;

connect_t connect_data = 
{
	.ONE_CHECK[0] = 'o',
	.ONE_CHECK[1] = 'j',
	.ONE_CHECK[2] = 'b',
	.ONE_CHECK[3] = 'k',
	.ONE_CHECK[4] = '?',
	.ONE_CHECK[5] = 0,
	.ONE_CHECK[6] = 0,
	.ONE_CHECK[7] = 0,
	
	.TWO_CHECK[0] = 'd',
	.TWO_CHECK[1] = 'd',
	.TWO_CHECK[2] = 'i',
	.TWO_CHECK[3] = 'u',
	.TWO_CHECK[4] = '.',
	.TWO_CHECK[5] = 0,
	.TWO_CHECK[6] = 0,
	.TWO_CHECK[7] = 0,	
};


/**
  * @brief          检测连接是否正常 发送与接收函数
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void send_check_package(const uint8_t *check)  
{	
	can2_tx_header.StdId = CAN2_CONNECT_CHECK_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)check[0];
    can2_tx_data[1] = (uint8_t)check[1];
    can2_tx_data[2] = (uint8_t)check[2];
    can2_tx_data[3] = (uint8_t)check[3];
    can2_tx_data[4] = (uint8_t)check[4];
    can2_tx_data[5] = (uint8_t)check[5];
    can2_tx_data[6] = (uint8_t)check[6];
    can2_tx_data[7] = (uint8_t)check[7];
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}
void receive_check_package_process(connect_t *connect_data, uint8_t aData[])  ///检测接收是否成功
{
	if(aData[0] == connect_data->TWO_CHECK[0] &&  \
	   aData[1] == connect_data->TWO_CHECK[1] &&  \
	   aData[2] == connect_data->TWO_CHECK[2] &&  \
	   aData[3] == connect_data->TWO_CHECK[3] &&  \
	   aData[4] == connect_data->TWO_CHECK[4] &&  \
	   aData[5] == connect_data->TWO_CHECK[5] &&  \
	   aData[6] == connect_data->TWO_CHECK[6] &&  \
	   aData[7] == connect_data->TWO_CHECK[7])
	{
		connect_data->receive_success_flag = 1;
	}
}
/**
  * @brief          发送检测数据包之后，循环等待接收回馈信息。连接正常则退出循环
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void check_connect(connect_t *connect_data)
{
	connect_data->receive_success_flag = 0;
	do
	{
		send_check_package(connect_data->ONE_CHECK);//向底盘发送one_check数据包确认连接
		LED_P6x8Str(16, 0, (uint8_t *)"connect fail!");
		BUZZER_ON();
//		vTaskDelay(10);
// break;
		HAL_Delay(10);
	}while(!connect_data->receive_success_flag);//等待接收底盘的two_check数据包
	connect_data->receive_success_flag = 0;//后面也会被置1底盘并未停止发送two 不影响 已不再使用
	BUZZER_OFF();
	LED_Fill(0x00);
	
}
/**
* @brief        can2获取rc数据 发送往底盘
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void can2_get_rc_data(connect_t *connect_data)
{
	connect_data->can2_rc_ctrl.control_mode = get_robot_control_mode() \
                                          | (shoot_control_data.magazine_control_flag<<7) \
                                          | (shoot_control_data.thumbwheel_move_flag<<6) \
                                          | (shoot_control_data.thumbwheel_contrary_flag<<5) \
	                                        | (shoot_control_data.thumbwheel_remote_flag<<4);
	connect_data->can2_rc_ctrl.work_mode = get_robot_work_mode();
	connect_data->can2_rc_ctrl.rc.ch2 = connect_data->rc_ctrl->rc.ch2 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 = connect_data->rc_ctrl->rc.ch3 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.mouse.key = connect_data->rc_ctrl->key.v;
}
void send_rc_to_chassis(can2_rc_ctrl_t *can2_rc_ctrl)  
{
	can2_tx_header.StdId = CAN2_CONNECT_RC_CTRL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(can2_rc_ctrl->control_mode);

    can2_tx_data[1] = (uint8_t)(can2_rc_ctrl->work_mode);

    can2_tx_data[2] = (uint8_t)(can2_rc_ctrl->rc.ch2 >> 8);
    can2_tx_data[3] = (uint8_t)(can2_rc_ctrl->rc.ch2);
    can2_tx_data[4] = (uint8_t)(can2_rc_ctrl->rc.ch3 >> 8);
    can2_tx_data[5] = (uint8_t)(can2_rc_ctrl->rc.ch3);
    can2_tx_data[6] = (uint8_t)(can2_rc_ctrl->mouse.key >> 8);
    can2_tx_data[7] = (uint8_t)(can2_rc_ctrl->mouse.key);

	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}
/**
* @brief        can2获取GYRO陀螺仪数据，发送往底盘
  * @author         
  * @param[in]      
  * @retval			
  * @note           
*/
extern gimbal_control_data_t gimbal_control_data;
void send_gyro_data_to_chassis(void)  
{
	can2_tx_header.StdId = CAN2_CONNECT_CM_GYRO_STD_ID;
  can2_tx_header.IDE = CAN_ID_STD;
  can2_tx_header.RTR = CAN_RTR_DATA;
  can2_tx_header.DLC = 0x08;

	int32_t set = my_abs((int16_t)(gimbal_control_data.gimbal_yaw_set*10));
	int32_t	fdb = my_abs((int16_t)(gimbal_control_data.gimbal_INS->yaw_angle*10));

  
  can2_tx_data[0] = (uint8_t)(set>>16);
  can2_tx_data[1] = (uint8_t)(set>>8);
  can2_tx_data[2] = (uint8_t)(set);
  can2_tx_data[3] = (uint8_t)(fdb>>16);
  can2_tx_data[4] = (uint8_t)(fdb>>8);
  can2_tx_data[5] = (uint8_t)(fdb);
  can2_tx_data[6] = gimbal_control_data.gimbal_yaw_set<0? 0xff : 0 ;	//符号位
  can2_tx_data[7] = gimbal_control_data.gimbal_INS->yaw_angle<0 ? 0xff:0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}

extern cali_chassis_t cali_chassis_pid;
/***************************************数据校准在C板cali_task****************************************/
/*****************************校准后的底盘电机PID数据发送给底盘********************************/
void set_chassis_rotate_pid_data_to_chassis(void)  
{
  	can2_tx_header.StdId = CAN2_CHASSIS_PID_ROTATE_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(((uint16_t)(cali_chassis_pid.rotate_pid.kp*10))>>8);
    can2_tx_data[1] = (uint8_t)((uint16_t)(cali_chassis_pid.rotate_pid.kp*10));
    can2_tx_data[2] = (uint8_t)(((uint16_t)(cali_chassis_pid.rotate_pid.ki*10))>>8);
    can2_tx_data[3] = (uint8_t)((uint16_t)(cali_chassis_pid.rotate_pid.ki*10));
	  can2_tx_data[4] = (uint8_t)(((uint16_t)(cali_chassis_pid.rotate_pid.kd*10))>>8);
    can2_tx_data[5] = (uint8_t)((uint16_t)(cali_chassis_pid.rotate_pid.kd*10));
    can2_tx_data[6] = 0;
    can2_tx_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1);
}
void set_chassis_CM_pid_data_to_chassis(void)  
{
	can2_tx_header.StdId = CAN2_CHASSIS_PID_CM_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.kp>>8);
    can2_tx_data[1] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.kp);
    can2_tx_data[2] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.ki>>8);
    can2_tx_data[3] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.ki);
	  can2_tx_data[4] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.kd>>8);
    can2_tx_data[5] = (uint8_t)((uint16_t)cali_chassis_pid.cm_pid.kd);
    can2_tx_data[6] = 0;
    can2_tx_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}
/**
  * @brief          接收数据函数 can2中断里
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void connect_cm_encode_process(connect_t *connect, uint8_t aData[])
{
	connect->cm1_encode = ((int16_t)aData[0] << 8) | (int16_t)aData[1];
	connect->cm2_encode = ((int16_t)aData[2] << 8) | (int16_t)aData[3];
	connect->cm3_encode = ((int16_t)aData[4] << 8) | (int16_t)aData[5];
	connect->cm4_encode = ((int16_t)aData[6] << 8) | (int16_t)aData[7];
}
void connect_cm_speed_process(connect_t *connect, uint8_t aData[])
{
	connect->cm1_rate = ((int16_t)aData[0] << 8) | (int16_t)aData[1];
	connect->cm2_rate = ((int16_t)aData[2] << 8) | (int16_t)aData[3];
	connect->cm3_rate = ((int16_t)aData[4] << 8) | (int16_t)aData[5];
	connect->cm4_rate = ((int16_t)aData[6] << 8) | (int16_t)aData[7];
}


/**
  * @brief          连接初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void connect_init(connect_t *connect_data)
{
	connect_data->rc_ctrl = get_rc_data_point();
	
//	vTaskSuspendAll();//挂起任务调度器
//	check_connect(connect_data);//如果未连接将无法退出循环，云台也不会发送rc数据
//	xTaskResumeAll();
}
/**
  * @brief         连接底盘任务
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
typedef union float_char_
{
	float a;
	uint8_t b[4];
}float_char;
void connect_task(void *argument)
{
	// TickType_t current_time = 0;

	connect_init(&connect_data);        ///遥控连接
	while(1)
	{
//		float_char f_c;
//		f_c.a = -1.3;
//		uint8_t cx_can2_tx_data[8];
//		cx_can2_tx_data[0] = f_c.b[0];
//		cx_can2_tx_data[1] = f_c.b[1];
//		cx_can2_tx_data[2] = f_c.b[2];
//		cx_can2_tx_data[3] = f_c.b[3];
//		can2_tx_header.StdId = 0xff;
//		can2_tx_header.IDE = CAN_ID_STD;
//		can2_tx_header.RTR = CAN_RTR_DATA;
//		can2_tx_header.DLC = 0x08;
//		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, cx_can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
		can2_get_rc_data(&connect_data);  // 用来赋值给下面语句数据来发送
		send_rc_to_chassis(&connect_data.can2_rc_ctrl);///通过can2发送遥控器数据给chassis
		if(Judge_data.robot_id < 10)    
	{
		Transmit_To_vision(0x22); //敌方是蓝色  
	}
	else
	{
		;
	}
		vTaskDelay(10);                        // 10ms一次
	}
}


connect_t *get_connect_data_point(void)
{
	return &connect_data;
}
