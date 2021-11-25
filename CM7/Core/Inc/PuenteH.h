/*
 * PuenteH.h
 *
 *  Created on: Nov 16, 2021
 *      Author: jorge
 */

#ifndef INC_PUENTEH_H_
#define INC_PUENTEH_H_

#include "stm32h7xx_hal.h"

typedef struct {
	TIM_HandleTypeDef* timer_h;

    // GPIO to set direction
    GPIO_TypeDef* pin_Forward_Group;
    uint32_t pin_Forward_num;

    // GPIO to set direction
    GPIO_TypeDef* pin_Reverse_Group;
    uint32_t pin_Reverse_num;

    // Reference to estimate the dutyCycle
    float voltage_Supplied;

    int Duty_Cycle;    // Default: 0x75
} DC_Motor_Controller;


//Constructor Wannabe
void DC_Motor_Controller_Init(DC_Motor_Controller* motor);

//Set a desired Output voltage based on the supply
// Velocity is a range from 0-1.0
void set_Relative_Velocity(DC_Motor_Controller* motor, float velocity);

//Change the way the motor turns.
void invert_Motor(DC_Motor_Controller* motor);

void stop_Motor(DC_Motor_Controller* motor);

void set_direction_Forward(DC_Motor_Controller* motor);

void set_direction_Backward(DC_Motor_Controller* motor);

#endif /* INC_PUENTEH_H_ */
