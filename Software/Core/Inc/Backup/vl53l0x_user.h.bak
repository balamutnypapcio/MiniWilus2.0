/*
 * vl53l0x_user.h
 *
 *  Created on: Nov 10, 2025
 *      Author: jakub
 */

#ifndef INC_VL53L0X_USER_H_
#define INC_VL53L0X_USER_H_

#include "main.h"
#include "vl53l0x_api.h"

void TOF_InitSensors(void);
uint16_t TOF_ReadDistance(uint8_t sensor_id);
void TOF_ReadAllDistances(uint16_t* distances_buffer);

extern VL53L0X_Dev_t tof_devices[4];

#endif /* INC_VL53L0X_USER_H_ */
