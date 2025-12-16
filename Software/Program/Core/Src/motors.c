#include "motors.h"
#include "tim.h"
#include <stdlib.h> 

static const HBridgeState STATE_FORWARD  = {GPIO_PIN_SET,   GPIO_PIN_RESET};
static const HBridgeState STATE_BACKWARD = {GPIO_PIN_RESET, GPIO_PIN_SET};
static const HBridgeState STATE_STOP     = {GPIO_PIN_RESET, GPIO_PIN_RESET};


static int8_t constrainSpeed(int8_t speed)
{
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < MIN_SPEED) return MIN_SPEED;
    return speed;
}

static HBridgeState determineMotorState(int8_t speed)
{
    if (speed > 0) {
        return STATE_FORWARD;
    }
    if (speed < 0) {
        return STATE_BACKWARD;
    }
    return STATE_STOP;
}




static uint16_t calculatePwmDuty(int8_t speed)
{
    return (uint16_t)((abs(speed) * PWM_PERIOD) / 100);
}

// --- Funkcje wysokiego poziomu (API) ---

void Motors_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void Motors_SetSpeed(Motor_t motor, int8_t speed)
{
    speed = constrainSpeed(speed);
    HBridgeState state = determineMotorState(speed);
    uint16_t pwmDuty = calculatePwmDuty(speed);
    if (motor == MOTOR_LEFT)
    {
        HAL_GPIO_WritePin(M1_OUT1_GPIO_Port, M1_OUT1_Pin, state.in1);
        HAL_GPIO_WritePin(M1_OUT2_GPIO_Port, M1_OUT2_Pin, state.in2);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmDuty);
    }
    else if (motor == MOTOR_RIGHT)
    {
        HAL_GPIO_WritePin(M2_OUT1_GPIO_Port, M2_OUT1_Pin, state.in1);
        HAL_GPIO_WritePin(M2_OUT2_GPIO_Port, M2_OUT2_Pin, state.in2);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmDuty);
    }
}

void Motors_Forward(int8_t speed)
{
    Motors_SetSpeed(MOTOR_LEFT, speed);
    Motors_SetSpeed(MOTOR_RIGHT, speed);
}

void Motors_Backward(int8_t speed)
{
    Motors_SetSpeed(MOTOR_LEFT, -speed);
    Motors_SetSpeed(MOTOR_RIGHT, -speed);
}

void Motors_TurnLeft(int8_t speed)
{
    Motors_SetSpeed(MOTOR_LEFT, -speed);
    Motors_SetSpeed(MOTOR_RIGHT, speed);
}

void Motors_TurnRight(int8_t speed)
{
    Motors_SetSpeed(MOTOR_LEFT, speed);
    Motors_SetSpeed(MOTOR_RIGHT, -speed);
}

void Motors_Stop(void)
{
    Motors_SetSpeed(MOTOR_LEFT, 0);
    Motors_SetSpeed(MOTOR_RIGHT, 0);
}