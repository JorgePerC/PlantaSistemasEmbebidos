/*
 * PuenteH.c
 *
 *  Created on: Nov 16, 2021
 *      Author: jorge
 */

#include "PuenteH.h"

void DC_Motor_Controller_Init(DC_Motor_Controller* motor){

	// Set an initial direction
	HAL_GPIO_WritePin(motor -> pin_Forward_Group, motor -> pin_Forward_num, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num, GPIO_PIN_RESET);

	//Initialize PWM signal
	HAL_TIM_PWM_Start(motor -> timer_h, TIM_CHANNEL_1);

	//Set velocity to 0:
	TIM3 -> CCR1 = 0;
}


void set_Relative_Velocity(DC_Motor_Controller* motor, float velocity){
	//uint16_t scaleFactor = (uint16_t) (velocity) * 100;
	uint16_t scaleFactor = (uint16_t) (motor -> voltage_Supplied / velocity) * 100;
	//TODO: Evitar llegar al 100% (CCR = 99)
	TIM3 -> CCR1 = scaleFactor;

}

//Change the way the motor turns.
void invert_Motor(DC_Motor_Controller* motor){
	// If the motor has been stopped, then toggle won't work, so... We add a case if this happens
	if (	!(HAL_GPIO_ReadPin(motor -> pin_Forward_Group, motor -> pin_Forward_num)
			&& HAL_GPIO_ReadPin(motor -> pin_Forward_Group, motor -> pin_Forward_num))	){

		HAL_GPIO_WritePin(motor -> pin_Forward_Group, motor -> pin_Forward_num, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num, GPIO_PIN_RESET);
	}
	//TODO: Should we try to avoid back EMF?
		// stop_motor(motor);
	//Invert Motors
	HAL_GPIO_TogglePin(motor -> pin_Forward_Group, motor -> pin_Forward_num);
	HAL_GPIO_TogglePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num);

}

void stop_Motor(DC_Motor_Controller* motor){
	TIM3 -> CCR1 = 0;
	// In case the last one doesn't work:
	HAL_GPIO_WritePin(motor -> pin_Forward_Group, motor -> pin_Forward_num, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num, GPIO_PIN_RESET);
}

void set_direction_Forward(DC_Motor_Controller* motor){
	HAL_GPIO_WritePin(motor -> pin_Forward_Group, motor -> pin_Forward_num, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num, GPIO_PIN_RESET);
}

void set_direction_Backward(DC_Motor_Controller* motor){
	HAL_GPIO_WritePin(motor -> pin_Forward_Group, motor -> pin_Forward_num, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor -> pin_Reverse_Group, motor -> pin_Reverse_num, GPIO_PIN_SET);
}

