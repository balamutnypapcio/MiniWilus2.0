/*
 * motors.h
 *
 *  Created on: Nov 10, 2025
 *      Author: jakub
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"
#include <stdlib.h>

void Motors_Init(void);

void Motors_SetSpeed(uint8_t motor_id, int8_t speed);

void Motors_Forward(int8_t speed);
void Motors_Backward(int8_t speed);
void Motors_TurnLeft(int8_t speed);
void Motors_TurnRight(int8_t speed);
void Motors_Stop(void);

#endif /* INC_MOTORS_H_ */
