/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#define CALI_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart1_rx_cali_data[CALI_DATA_PACKAGE_SIZE];
extern uint8_t usart1_tx_cali_data[CALI_DATA_PACKAGE_SIZE];
#define DBUS_RX_BUF_NUM 36
extern uint8_t usart3_remote_data[DBUS_RX_BUF_NUM];
#define VISION_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart6_rx_vision_data[VISION_DATA_PACKAGE_SIZE];
extern uint8_t usart6_tx_vision_data[VISION_DATA_PACKAGE_SIZE];
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void usart_Receive_DMA(void);
void Transmit_To_vision(uint8_t enemy);
/**********USART1**********/
void usart1_send_data(uint8_t *tx_data_package);
/***********USART3******************/
void usart3_reset(void);
/************USART6**********************/
uint8_t TX2_MES(void);
void TX2_RX_INT(void);

typedef __packed struct 
{ 
	uint8_t frame_seq;
	uint8_t shoot_mode;
	uint8_t shoot_speed;
	float pitch_dev;
	float yaw_dev;
	int16_t rail_speed;
	uint8_t gimbal_mode;
	
}_tx2_feedback_data;

typedef __packed struct 
{
	uint8_t frame_seq;
	uint16_t shoot_mode;
	
	float pitch_dev;//pitch位置  -5-20
	float yaw_dev;//-5-20 deviation
	int16_t rail_speed;
	uint8_t gimbal_mode;
	float pitch_old_dev;//之前pitch位置
	float yaw_old_dev;
	float Target_distance;//识别物体距离 1m=1000
	float Target_old_distance;
	float pitch_dev_err;//pitch位置误差
	float yaw_dev_err;
	

	float yaw_compensate;			// 水平位置偏差补偿
	float pitch_compensate;			// 竖直位置偏差补偿
	float Gravity_compensation;		// 重力补偿
	float G_dev;
	float pitch_distance_compensation;//pitch距离补偿
	float yaw_INS_compensation;

}_tx2_control_data;

typedef struct
{
	
	uint8_t check_top_byte;
	uint8_t check_bottom_byte;
	uint8_t pid_type_data;

	
	uint8_t receive_success_flag; 
	uint8_t beep_flag;            
	
} vision_t;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
