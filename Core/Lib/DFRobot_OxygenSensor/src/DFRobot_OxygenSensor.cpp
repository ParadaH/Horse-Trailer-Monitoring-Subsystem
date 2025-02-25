/*!
 * @file DFRobot_OxygenSensor.cpp
 * @brief Define the basic struct of DFRobot_OxygenSensor class, the implementation of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.0.1
 * @date 2022-08-02
 * @url https://github.com/DFRobot/DFRobot_OxygenSensor
 */
#include "DFRobot_OxygenSensor.h"

DFRobot_OxygenSensor::DFRobot_OxygenSensor(I2C_HandleTypeDef *pI2C) {
	this->_pI2C = pI2C;
}

DFRobot_OxygenSensor::~DFRobot_OxygenSensor() {
	this->_pI2C = NULL;
}

bool DFRobot_OxygenSensor::begin(uint8_t addr) {
	this->_addr = addr;
	if (HAL_I2C_IsDeviceReady(_pI2C, _addr << 1, 3, HAL_MAX_DELAY) == HAL_OK) {
		return true;
	}
	return false;
}

void DFRobot_OxygenSensor::readFlash() {
	uint8_t value = 0;
	uint8_t reg = GET_KEY_REGISTER;

	HAL_I2C_Master_Transmit(_pI2C, _addr << 1, &reg, 1, HAL_MAX_DELAY);
	HAL_Delay(50);

	HAL_I2C_Master_Receive(_pI2C, _addr << 1, &value, 1, HAL_MAX_DELAY);

	if (value == 0) {
		this->_Key = 20.9 / 120.0;
	} else {
		this->_Key = (float) value / 1000.0;
	}

}

void DFRobot_OxygenSensor::i2cWrite(uint8_t reg, uint8_t data) {
	uint8_t buffer[2] = { reg, data };
	HAL_I2C_Master_Transmit(_pI2C, _addr << 1, buffer, 2, HAL_MAX_DELAY);
}

void DFRobot_OxygenSensor::calibrate(float vol, float mv) {
	uint8_t keyValue = vol * 10;
	if (mv < 0.000001 && mv > (-0.000001)) {
		i2cWrite(USER_SET_REGISTER, keyValue);
	} else {
		keyValue = (vol / mv) * 1000;
		i2cWrite(AUTUAL_SET_REGISTER, keyValue);
	}
}

float DFRobot_OxygenSensor::getOxygenData(uint8_t collectNum) {
	uint8_t rxbuf[3] = { 0 };
	uint8_t reg = OXYGEN_DATA_REGISTER;
	static uint8_t i = 0, j = 0;

	readFlash();

	if (collectNum > 0) {

		for (j = collectNum - 1; j > 0; j--) {
			oxygenData[j] = oxygenData[j - 1];
		}
		HAL_I2C_Master_Transmit(_pI2C, _addr << 1, &reg, 1, HAL_MAX_DELAY);
		HAL_Delay(100);
		HAL_I2C_Master_Receive(_pI2C, _addr << 1, rxbuf, 3, HAL_MAX_DELAY);

		oxygenData[0] = (_Key)
				* (((float) rxbuf[0]) + ((float) rxbuf[1] / 10.0)
						+ ((float) rxbuf[2] / 100.0));

		if (i < collectNum)
			i++;
		return getAverageNum(oxygenData, i);
	} else {
		return -1.0;
	}
}

float DFRobot_OxygenSensor::getAverageNum(float bArray[], uint8_t len) {
	uint8_t i;
	double bTemp = 0;
	for (i = 0; i < len; i++) {
		bTemp += bArray[i];
	}
	return bTemp / (float) len;
}
