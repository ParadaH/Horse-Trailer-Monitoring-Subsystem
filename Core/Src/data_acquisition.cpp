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
#include "data_acquisition.h"
#include "../Lib/DFRobot_OxygenSensor/src/DFRobot_OxygenSensor.h"
#include "../Lib/Seeed_SCD30/SCD30.h"

#define PMS5003_FRAME_SIZE 32  // Number of bytes in data packet from PMs sensor

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;
extern DFRobot_OxygenSensor oxygenSensor;
extern SCD30 scd30;
extern uint16_t CO2[3];
extern uint16_t PMs[3]; //pm1_0, pm2_5, pm10;
char buffer[16];
char dataBuffer[32];
const float RL = 9.62;  // Measured value of the pull-up resistor (10 kΩ)
const float R0 = 1.0; // Needs to be calibrated in fresh air

typedef struct {
    float CO;
    float O2;
    float NOx;
    float CO2;
    float PMs[3];
} Sensors_t;

extern Sensors_t sensors;

extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];
extern uint32_t mailbox;
// UART5 - communication with USB-UART converter and PC

void measure_CO(void){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 500);
	float COsenosorValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	float Rs = ((1023.0 / COsenosorValue) - 1) * RL;
	float ratio = Rs/R0;
	float ppm = powf(10, (log10f(ratio) - 1.7) / -1.5);
	sensors.CO = ppm;

	/* When using CAN bus data is sent here */
	TxHeader.StdId = 0x01;
	uint16_t CO = ppm * 100;
	TxData[0] = (uint16_t) 	CO & 0xFF;
	TxData[1] = (uint16_t) (CO >> 8) & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailbox);

	/* When using ST-Link data is sent here */
	sprintf(buffer, "000%.4f\r\n", ppm);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_O2(void){
	float oxygenConcentration = oxygenSensor.getOxygenData(10);
	sensors.O2 = oxygenConcentration;

	/* When using CAN bus data is sent here */
	TxHeader.StdId = 0x02;
	uint16_t O2 = oxygenConcentration * 100; // Scaling data from integer to float
	TxData[0] = (uint16_t) 	O2 & 0xFF;
	TxData[1] = (uint16_t) (O2 >> 8) & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailbox);

	/* When using ST-Link data is sent here */
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
	sensors.NOx = output;

	/* When using CAN bus data is sent here */
	TxHeader.StdId = 0x02;
	uint16_t NOx = output * 100;
	TxData[0] = (uint16_t) 	NOx & 0xFF;
	TxData[1] = (uint16_t) (NOx >> 8) & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailbox);

	/* When using ST-Link data is sent here */
	sprintf(buffer, "002%.4f\r\n", output);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_CO2(void){
	float sensorData[3];
	if (scd30.isAvailable()) {
		scd30.getCarbonDioxideConcentration(sensorData);
	}
	sensors.PMs[0] = sensorData[0];
	sensors.PMs[1] = sensorData[1];
	sensors.PMs[2] = sensorData[2];

	/* When using CAN bus data is sent here */
	TxHeader.StdId = 0x03;
	CO2[0] = sensors.PMs[0] * 100;
	TxData[0] = (uint16_t) 	CO2[0] & 0xFF;
	TxData[1] = (uint16_t) (CO2[0] >> 8) & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailbox);

	/* When using ST-Link data is sent here */
	sprintf(buffer, "003%.4f\r\n", sensorData[0]);
	HAL_UART_Transmit(&huart5, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
	memset(buffer, 0, sizeof(buffer));
}

void measure_PMs(uint8_t *data){
	char txBuffer[32];

	if (data[0] == 0x42 && data[1] == 0x4D){
		sensors.PMs[0] = (data[10] << 8) | data[11];
		sensors.PMs[1] = (data[12] << 8) | data[13];
		sensors.PMs[2] = (data[14] << 8) | data[15];
	}

	/* When using CAN bus data is sent here */
	TxHeader.StdId = 0x04;
	PMs[0] = sensors.PMs[0] * 100;
	PMs[1] = sensors.PMs[1] * 100;
	PMs[2] = sensors.PMs[2] * 100;
	TxData[0] = (uint16_t) 	CO2[0] & 0xFF;
	TxData[1] = (uint16_t) (CO2[0] >> 8) & 0xFF;
	TxData[2] = (uint16_t) 	CO2[1] & 0xFF;
	TxData[3] = (uint16_t) (CO2[1] >> 8) & 0xFF;
	TxData[4] = (uint16_t) 	CO2[2] & 0xFF;
	TxData[5] = (uint16_t) (CO2[2] >> 8) & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailbox);

	/* When using ST-Link data is sent here */

	sprintf(txBuffer, "004%.4f\r\n", sensors.PMs[0]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));

	sprintf(txBuffer, "005%.4f\r\n", sensors.PMs[1]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));

	sprintf(txBuffer, "006%.4f\r\n", sensors.PMs[2]);
	HAL_UART_Transmit(&huart5, (uint8_t*) txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
	memset(txBuffer, 0, sizeof(txBuffer));
}



