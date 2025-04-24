/*
 * app_callback.c
 *
 *  Created on: Apr 24, 2025
 *      Author: danilo
 */
#include "main.h"

extern uint32_t g_counter;
extern uint32_t g_channel_1_state;
extern uint32_t g_channel_2_state;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  g_counter = 0;
  g_channel_1_state = 32000;
  g_channel_2_state = 16000;

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
	  g_channel_1_state = 0;
  }

  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
  	  g_channel_2_state = 0;
    }

}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

}
