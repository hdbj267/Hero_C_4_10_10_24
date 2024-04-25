/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe  
 * @Teammate��
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description: 
 * @Note:       
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "can2_app.h"
#include "monitor_task.h"
#include "connect_task.h"

extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;

motor_msg_t yaw_motor_msg = {0};//can2 
motor_msg_t pitch_motor_msg = {0};
motor_msg_t pitch1_motor_msg = {0};

ext_Judge_data_t Judge_data;
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    Ĭ�ϵ����can����Ƶ��Ϊ1KHZ       
  */
static void gimbal_motor_msg_process(motor_msg_t *m,  uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//��������
	m->encoder.last_raw_value = m->encoder.raw_value; 
	if(m->encoder.start_flag==0)//�ϵ�ɼ�ԭʼ�Ƕ�
	{
		m->encoder.ecd_bias = (aData[0]<<8)|aData[1];//��ʼλ��
		m->encoder.last_raw_value = (aData[0]<<8)|aData[1];
		m->encoder.raw_value = m->encoder.last_raw_value;
		m->encoder.start_flag = 1;
	}
	else
	{
		m->encoder.raw_value = (aData[0]<<8)|aData[1];
	}
	
	m->encoder.diff = m->encoder.raw_value - m->encoder.last_raw_value;
	if(m->encoder.diff < -6000)//���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�                         
	{                          //7500���ݿ������ڿɵ�������֤һ������������ ת�ӻ�е�Ƕȱ仯С��8191-7500=691���� 
		m->encoder.round_cnt ++;
		m->encoder.ecd_raw_rate = m->encoder.diff + 8192;
	}
	else if(m->encoder.diff > 6000)
	{
		m->encoder.round_cnt --;
		m->encoder.ecd_raw_rate = m->encoder.diff - 8192;
	}
	else
	{
		m->encoder.ecd_raw_rate = m->encoder.diff;
	}
	//����õ��Ƕ�ֵ����Χ���������
		m->encoder.ecd_angle = m->encoder.raw_value *360/8192;

	
	m->encoder.rate_buf[m->encoder.buf_count++] = m->encoder.ecd_raw_rate;
	if(m->encoder.buf_count == RATE_BUF_SIZE)
	{
		m->encoder.buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		m->encoder.filter_rate_sum += m->encoder.rate_buf[i];
	}
	m->encoder.filter_rate = (int32_t)(m->encoder.filter_rate_sum/RATE_BUF_SIZE);	
	/*---------------------�Ǳ���������------------------------*/
	m->speed_rpm = (uint16_t)(aData[2] << 8 | aData[3]);     
	m->given_current = (uint16_t)(aData[4] << 8 | aData[5]); 
	m->temperate = aData[6];     

}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
 
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get gimbal control 
		case CAN2_YAW_MOTOR_STD_ID:
		{
			gimbal_motor_msg_process(&yaw_motor_msg ,aData); 
			monitor.yaw_motor.time = xTaskGetTickCount();
		}break;
		case CAN2_PITCH_MOTOR_STD_ID:
		{
			gimbal_motor_msg_process(&pitch_motor_msg ,aData); 
			monitor.pitch_motor.time = xTaskGetTickCount();			
		}break;
		case CAN2_PITCH1_MOTOR_STD_ID:
		{
			gimbal_motor_msg_process(&pitch1_motor_msg ,aData); 
			monitor.pitch_motor.time = xTaskGetTickCount();			
		}break;
		//get rc control 
		// case CAN2_CONNECT_CHECK_STD_ID:
		// {
		// 	receive_check_package_process(&connect_data, aData);
		// }break;
		//get chassis motor encode
		// case CAN2_CONNECT_CM_ENCODE_STD_ID:
		// {
		// 	connect_cm_encode_process(&connect_data, aData);
		// }break;
		//get chassis motor rate
		case CAN2_CONNECT_CM_SPEED_STD_ID:
		{
			connect_cm_speed_process(&connect_data, aData);
		}break;
		//��ò���ϵͳ����Ϣ   *hyj 
		case CAN2_SHOOT_42mm_ID:
		{
			shoot_42mm_mag(&Judge_data ,aData); 
		}break;
		case CAN2_SHOOT_JUDGE_ID:
		{
			shoot_judge_process(&Judge_data ,aData); 
		}break;

		default: break;
	}

}

void shoot_42mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[])
{
	Judge_data->shooter_id1_42mm_cooling_rate   = ((int16_t)aData[0] << 8) | (int16_t)aData[1];
	Judge_data->shooter_id1_42mm_cooling_limit  = ((int16_t)aData[2] << 8) | (int16_t)aData[3];
	Judge_data->shooter_id1_42mm_speed_limit    = ((int16_t)aData[4] << 8) | (int16_t)aData[5];
	Judge_data->shooter_id1_42mm_cooling_heat   = ((int16_t)aData[6] << 8) | (int16_t)aData[7];
}
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[])
{
	Judge_data->bullet_speed = ((float)((aData[0]<<8)|aData[1]))/100;
	Judge_data->hurt_type = aData[2];
	Judge_data->mains_power_shooter_output = aData[3];
	Judge_data->robot_id = aData[4];
}

/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note  �����̵���巢��ָ�ID��Ϊ0x200?����̷���IDΪ0x201-0x204
		   -16384 ~ +16384 ��Ӧ-20A ~ +20A �������ƾ���0.00122A
		   ����ķ���Ƶ��Ϊ1KHZ��ת��λ��0-8191 ��Ӧ 360Ҳ�Ǿͷֱ���Ϊ0.04394531..(ת�ӵ�)
		   ���ٱ�Ϊ1��19 Ҳ���Ƕ�Ӧת��ת��Ϊ�ֱ���Ϊ0.002312911       
  */
// void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq)  
// {	
// 	can2_tx_header.StdId = CAN2_GIMBAL_STD_ID;
//     can2_tx_header.IDE = CAN_ID_STD;
//     can2_tx_header.RTR = CAN_RTR_DATA;
//     can2_tx_header.DLC = 0x08;
    
//     can2_tx_data[0] = (uint8_t)(yaw_iq1 >> 8);
//     can2_tx_data[1] = (uint8_t)yaw_iq1;
// 	can2_tx_data[2] = (uint8_t)(yaw_iq2 >> 8);
//     can2_tx_data[3] = (uint8_t)yaw_iq2;
//     can2_tx_data[4] = (uint8_t)(pitch_iq >> 8);
//     can2_tx_data[5] = (uint8_t)pitch_iq;
//     can2_tx_data[6] = (uint8_t)(0 >> 8);
//     can2_tx_data[7] = (uint8_t)0;
// 	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
// }
/*****************************2��pitch�ĵ�����ƺ���******************************************/
void set_gimbal_behaviour(int16_t yaw_iq1, int16_t yaw_iq2, int16_t pitch_iq1,int16_t pitch_iq2)  
{	
	can2_tx_header.StdId = CAN2_GIMBAL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(yaw_iq1 >> 8);
    can2_tx_data[1] = (uint8_t)yaw_iq1;
	can2_tx_data[2] = (uint8_t)(yaw_iq2 >> 8);
    can2_tx_data[3] = (uint8_t)yaw_iq2;
    can2_tx_data[4] = (uint8_t)(pitch_iq1 >> 8);
    can2_tx_data[5] = (uint8_t)pitch_iq1;
    can2_tx_data[6] = (uint8_t)(pitch_iq2 >> 8);
    can2_tx_data[7] = (uint8_t)pitch_iq2;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void set_gimbal_stop(void)
{
    can2_tx_header.StdId = CAN2_GIMBAL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(0 >> 8);
    can2_tx_data[1] = (uint8_t)0;
    can2_tx_data[2] = (uint8_t)(0 >> 8);
    can2_tx_data[3] = (uint8_t)0;
    can2_tx_data[4] = (uint8_t)(0 >> 8);
    can2_tx_data[5] = (uint8_t)0;
    can2_tx_data[6] = (uint8_t)(0 >> 8);
    can2_tx_data[7] = (uint8_t)0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
motor_msg_t *get_yaw_motor_msg_point(void)
{
	return &yaw_motor_msg;
}
motor_msg_t *get_pitch_motor_msg_point(void)
{
	return &pitch_motor_msg;
}
motor_msg_t *get_pitch1_motor_msg_point(void)
{
return &pitch1_motor_msg;
}

