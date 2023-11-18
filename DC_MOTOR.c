

#include"DC_MOTOR.h"

#include"main.h"

extern TIM_HandleTypeDef htim2;


void DC_MOTOR_INTI()
{
	/* Initialize the PWM signal to control the motor speed */

	HAL_TIM_PWM_Start(&DC_MOTOR_TIMER, DC_MOTOR_CHANNEL );
}

void DC_MOTOR_SetSpeed(uint16_t Speed)
{
	/* Setting up the compare value to control the speed of the motor */

	__HAL_TIM_SET_COMPARE(&DC_MOTOR_TIMER,DC_MOTOR_CHANNEL,Speed);
}

void DC_MOTOR_SetDelay(uint32_t Milli_Sec)
{
	/* Performing delay for the motor to control the speed */
	HAL_Delay(Milli_Sec);
}

