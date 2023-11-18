/*
 * JOYSTICK.c
 *
 *  Created on: Nov 10, 2023
 *      Author: Ahmed
 */


#include"main.h"

uint32_t  DAT[2];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


#define PADAL_ADC  hadc1

void PADAL_START()
{

	/* Operate ADC_CAN_MOOD to read from both two channels and send data to DAT_ARRAY
	 *
	 * As data is sent to DAT array using DMA so interrupt is fired in DMA_ISR
	 *
	 * Store Readings of X-AXIS >> DAT[0]
	 * Store readings of Y-AXIS >> DAT[1]
	 *  */
//   HAL_ADC_Start_DMA(&PADAL_ADC, DAT , 2);

	HAL_ADC_Start_DMA(&PADAL_ADC, DAT, 2);

}


uint32_t PADAL_X_AXIS_READ()
{
	return DAT[0];
}

uint32_t PADAL_Y_AXIS_READ()
{
	return DAT[1];
}

