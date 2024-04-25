/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "oled.h"
#include "stdio.h"
uint8_t usart1_rx_cali_data[CALI_DATA_PACKAGE_SIZE];
uint8_t usart1_tx_cali_data[CALI_DATA_PACKAGE_SIZE];
uint8_t usart3_remote_data[DBUS_RX_BUF_NUM];
uint8_t usart6_rx_vision_data[VISION_DATA_PACKAGE_SIZE];
uint8_t usart6_tx_vision_data[VISION_DATA_PACKAGE_SIZE];
uint8_t PCrx[16];
unsigned char PCyaw[4],PCpitch[4];
_tx2_control_data control_data=
{
	.pitch_old_dev=6,//6为大概中间位置   防止开自瞄第一次识别到数据，数据不行，pitch_dev=0猛抬头
	.yaw_old_dev=14,
};
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 460800;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
    
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void usart_Receive_DMA(void)
{
  // HAL_UART_Receive_DMA(&huart1,usart1_rx_cali_data,30);   
  HAL_UART_Receive_DMA(&huart3,usart3_remote_data,18);
  // HAL_UART_Receive_DMA(&huart6,PCrx,16);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,PCrx,16);
  // HAL_UART_Receive_IT(&huart1,PCrx,16);

}
void Transmit_To_vision(uint8_t enemy)
{
    static uint8_t Sent_Data[8];
    Sent_Data[0] = 0x66; 
	Sent_Data[1] = enemy;
    Sent_Data[2] = enemy;
    Sent_Data[3] = enemy;
    Sent_Data[4] = enemy;
    Sent_Data[5] = enemy;
    Sent_Data[6] = enemy;
    Sent_Data[7] = 0x88;   
	HAL_UART_Transmit(&huart1, Sent_Data, sizeof(Sent_Data), 1000);
}
void usart1_send_data(uint8_t *tx_data_package)
{
	HAL_UART_Transmit(&huart1, tx_data_package, 30/*sizeof(usart1_tx_cali_data)*/, 1000);
}
	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
//??printf
#if 1
#pragma import(__use_no_semihosting)               
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;    
FILE __stdin;   
void _sys_exit(int x)  
{ 
	x = x; 
} 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
	return ch;
  
}
#endif 

void usart3_reset(void)
{
	MX_USART3_UART_Init();
  HAL_UART_Receive_DMA(&huart3,usart3_remote_data,18);
}

void TX2_RX_INT(void)
{
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)
	{  
     static uint32_t i;
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);             //ć¸é¤çŠşé˛ćĽĺć ĺż
      i = (&huart1)->Instance->SR;
      i = (&huart1)->Instance->DR;
      HAL_UART_DMAStop(&huart1);          						//ĺć­˘DMAćĽĺ
      i = (&hdma_usart1_rx)->Instance->NDTR;
      TX2_MES();                       							  //ä˝żč˝ćĽćś			
      memset(PCrx,0,sizeof(PCrx));
      HAL_UART_Receive_DMA(&huart1,PCrx,16);          //éć°ä˝żč˝DMAćĽćś
  }
 
  // if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=RESET)
	// {
  //   __HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_RXNE);     //ć¸ć ĺż?
	// 	TX2_MES();                       							     		
	// 	memset(PCrx,0,sizeof(PCrx));
	// 	HAL_UART_Receive_IT(&huart1,PCrx,16);             //éć°ä˝żč˝ćĽćś
  // }
}


float view_float(float amt, float low, float high) //限制函数
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

extern uint8_t view_control_flag;
uint8_t view_lost_cnt=0;
uint8_t TX2_MES(void)
{
		
	if((PCrx[0] == 0x66) && (PCrx[15] == 0x88))     
	{
		control_data.yaw_old_dev=control_data.yaw_dev;
		control_data.pitch_old_dev=control_data.pitch_dev;
		control_data.Target_old_distance=control_data.Target_distance;
		
		control_data.frame_seq = (short)(PCrx[1]);	
		control_data.shoot_mode = (uint16_t)(PCrx[3] << 8 | PCrx[2]);
		control_data.Target_distance = control_data.shoot_mode;
		//pitch
		*((unsigned char *)(&PCpitch))     = PCrx[8];
		*((unsigned char *)(&PCpitch) + 1) = PCrx[9];
		*((unsigned char *)(&PCpitch) + 2) = PCrx[10];
		*((unsigned char *)(&PCpitch) + 3) = PCrx[11];
		control_data.pitch_dev=*(float *)PCpitch;
		//yaw 
		*((unsigned char *)(&PCyaw))     = PCrx[4];
		*((unsigned char *)(&PCyaw) + 1) = PCrx[5];
		*((unsigned char *)(&PCyaw) + 2) = PCrx[6];
		*((unsigned char *)(&PCyaw) + 3) = PCrx[7];
		control_data.yaw_dev=*(float *)PCyaw;

		control_data.rail_speed = (short)(PCrx[12] << 8 | PCrx[13]);
		control_data.gimbal_mode = (short)(PCrx[14]);

		if(	(control_data.pitch_dev<=-5)||(	control_data.pitch_dev>=25))
			control_data.pitch_dev=control_data.pitch_old_dev;
		
		if(	(control_data.yaw_dev<=-5)||(	control_data.yaw_dev>=25))
			control_data.yaw_dev=control_data.yaw_old_dev;

		control_data.yaw_dev_err=control_data.yaw_dev-control_data.yaw_old_dev;
		control_data.pitch_dev_err=control_data.pitch_dev-control_data.pitch_old_dev;
		
		control_data.yaw_dev=control_data.yaw_old_dev+view_float(control_data.yaw_dev_err,-1,1);//降低变换率 防止变换过大，大摆头
		control_data.pitch_dev=control_data.pitch_old_dev+view_float(control_data.pitch_dev_err,-1.5,1.5);
		
		 control_data.Target_distance =0.75f* control_data.Target_distance +0.25f* control_data.Target_old_distance ;
		
//		if ( ((control_data.yaw_dev == control_data.yaw_old_dev)||(control_data.pitch_dev == control_data.pitch_old_dev)) \
//    && ((control_data.yaw_dev)< 0.2f) && ((control_data.yaw_dev)> -0.2f)&&((control_data.pitch_dev)< 0.2f)&& ((control_data.pitch_dev)> -0.2f) && view_lost_cnt <1)
//		{
//			view_lost_cnt++;
//		}
//		else
//		{
//			view_lost_cnt=0;
//		}

		// if( (view_lost_cnt>=2) || \
    // ( ((control_data.yaw_dev == control_data.yaw_old_dev)||(control_data.pitch_dev == control_data.pitch_old_dev)) \
    // && ((((control_data.yaw_dev) > 0.2f)||(control_data.yaw_dev< -0.2f)))&&((control_data.pitch_dev> 0.2f)|| ((control_data.pitch_dev) < -0.2f))) )
		// {
		// 	view_control_flag=0;
		// 	view_lost_cnt=2;
		// }
		// else
		// {
		// 	view_control_flag=1;
    //   view_lost_cnt = 2 ;
		// }

    if((control_data.yaw_dev == control_data.yaw_old_dev)||(control_data.pitch_dev == control_data.pitch_old_dev))
		{
			view_control_flag=0;
      
			// view_lost_cnt=2;
		}
		else
		{
			view_control_flag=1;
      // view_lost_cnt = 2 ;
		}
		if(control_data.Target_distance==0)
		{
			view_control_flag=0;
		}

		

	}
	return 1;			
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
