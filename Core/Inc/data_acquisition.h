/*
 * data_acquisition.h
 *
 *  Created on: Feb 19, 2025
 *      Author: Hubert P
 */

#ifndef SRC_DATA_ACQUISITION_H_
#define SRC_DATA_ACQUISITION_H_


void measure_CO(void);
void measure_O2(void);
void measure_NOx(void);
void measure_CO2(void);
void measure_PMs(uint8_t *data);

#endif /* SRC_DATA_ACQUISITION_H_ */
