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
#include "../Lib/Seeed_SCD30/SCD30.h"

#define PMS5003_FRAME_SIZE 32  // Number of bytes in data packet from PMs sensor

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;
extern DFRobot_OxygenSensor oxygenSensor;
extern SCD30 scd30;
extern float CO2[3];
char buffer[16];
char dataBuffer[32];
const float RL = 9.62;  // Measured value of the pull-up resistor (10 kΩ)
const float R0 = 1.0; // Needs to be calibrated in fresh air
extern float pm[3]; //pm1_0, pm2_5, pm10;
// UART5 - communication with USB-UART converter and PC app

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
	memset(buffer, 0, sizeof(buffer));
}

void measure_O2(void){
	float oxygenConcentration = oxygenSensor.getOxygenData(10);
	sprintf(buffer, "001%.4f\r\n", oxygenConcentration);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_NOx(void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	float NOxsensorValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float sensedVoltage = (NOxsensorValue / 1023.0) * 5.0;
	float output = pow(sensedVoltage, 3)*(-0.00130116)+pow(sensedVoltage, 2)*(0.03712166)
			+sensedVoltage*(-0.42411996)+(-0.00329962);
	sprintf(buffer, "002%.4f\r\n", output);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_CO2(void){
	float sensorData[3];
	if (scd30.isAvailable()) {
		scd30.getCarbonDioxideConcentration(sensorData);
	}
	CO2[0] = sensorData[0];
	CO2[1] = sensorData[1];
	CO2[2] = sensorData[2];
	sprintf(buffer, "003%.4f\r\n", sensorData[0]);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_PMs(uint8_t *data){
	char txBuffer[32];

	if (data[0] == 0x42 && data[1] == 0x4D){
	  	pm[0] = (data[10] << 8) | data[11];
		pm[1] = (data[12] << 8) | data[13];
		pm[2] = (data[14] << 8) | data[15];
	}

	sprintf(txBuffer, "004%.4f\r\n", pm[0]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));

	sprintf(txBuffer, "005%.4f\r\n", pm[1]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));

	sprintf(txBuffer, "006%.4f\r\n", pm[2]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));
}



