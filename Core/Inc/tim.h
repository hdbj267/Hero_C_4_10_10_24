/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_TIM10_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
/**********TIM1******************/
 void pwm1_on(uint32_t compare);
 void pwm1_off(void);
/**********TIM3*****************/
extern volatile uint64_t freertos_run_time;
void freertos_run_tim3_configuration(void);//freertos获取task_time的定时器函数
uint32_t get_tim3_micros(void);
/********TIM4**************/
#define BUZZER_ON()      buzzer_on(50)
#define BUZZER_OFF()     buzzer1_off()
void buzzer_on(uint32_t val);
void buzzer_off(void);
void buzzer1_on(uint16_t psc, uint16_t pwm);
void buzzer1_off(void);
/********TIM6********************/
void tim6_configuration(void);
/********TIM10********************/
void imu_pwm_set(uint16_t pwm);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
