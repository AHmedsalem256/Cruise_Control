


#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "main.h"
/* Some important private macros to determine the hardware connections of the Timer */

#define DC_MOTOR_CHANNEL  TIM_CHANNEL_1
#define DC_MOTOR_TIMER    htim2
/*******************************************************************************/


/* Basic api's for the DC-MOTOR driver */

void DC_MOTOR_INTI();

void DC_MOTOR_SetSpeed(uint16_t Speed);

void DC_MOTOR_SetDelay(uint32_t Milli_Sec);

void PEDAL_MANUAL_MOTOR_CONTROL();

#endif
