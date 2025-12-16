#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"

typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor_t;

typedef struct {
    GPIO_PinState in1;
    GPIO_PinState in2;
} HBridgeState;

#define MAX_SPEED 100
#define MIN_SPEED -100
#define PWM_PERIOD 499

void Motors_Init(void);
void Motors_SetSpeed(Motor_t motor, int8_t speed);
void Motors_Forward(int8_t speed);
void Motors_Backward(int8_t speed);
void Motors_TurnLeft(int8_t speed);
void Motors_TurnRight(int8_t speed);
void Motors_Stop(void);

#endif /* INC_MOTORS_H_ */