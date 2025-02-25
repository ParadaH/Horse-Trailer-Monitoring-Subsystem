/*
 * data_acquisition.c
 *
 *  Created on: Feb 19, 2025
 *      Author: Hubert P
 */

#include "main.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "../Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart5;
extern DFRobot_OxygenSensor oxygenSensor;
extern float oxygen;
char buffer[16];
const float RL = 9.62;  // Measured value of the pull-up resistor (10 kΩ)
const float R0 = 1.0; // Needs to be calibrated in fresh air

void measure_CO(void){

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	float COsenosorValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float Rs = ((1023.0 / COsenosorValue) - 1) * RL;
	float ratio = Rs/R0;
	float ppm = powf(10, (log10f(ratio) - 1.7) / -1.5);

	sprintf(buffer, "000%.4f\r\n", ppm);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}

void measure_O2(void){
	float oxygenConcentration = oxygenSensor.getOxygenData(10);
	oxygen = oxygenSensor.getOxygenData(10);
	sprintf(buffer, "001%.4f\r\n", oxygenConcentration);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}

