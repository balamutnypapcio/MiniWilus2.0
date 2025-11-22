/*
 * motors.c
 *
 *  Created on: Nov 10, 2025
 *      Author: jakub
 */
#include "main.h" // Dołączamy main.h, aby mieć dostęp do wszystkich definicji pinów
#include "motors.h"
#include "tim.h"  // Potrzebujemy uchwytu htim3

// --- Implementacje funkcji ---

void Motors_Init(void)
{

    // Uruchomienie kanałów PWM dla obu silników
    // TIM_CHANNEL_4 jest na pinie PWM1 (PB1)
    // TIM_CHANNEL_1 jest na pinie PWM2 (PB4)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void Motors_SetSpeed(uint8_t motor_id, int8_t speed)
{
    // Walidacja danych wejściowych
    if (speed < -100) speed = -100;
    if (speed > 100) speed = 100;

    // Obliczenie wypełnienia PWM
    // Zakładamy Counter Period (ARR) = 499
    uint16_t pwm_duty = (uint16_t)((abs(speed) * 499) / 100);

    if (motor_id == 1) // --- Sterowanie Silnikiem 1 (M1) ---
    {
        if (speed > 0) { // Jazda do przodu
            HAL_GPIO_WritePin(M1_OUT1_GPIO_Port, M1_OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M1_OUT2_GPIO_Port, M1_OUT2_Pin, GPIO_PIN_RESET);
        } else if (speed < 0) { // Jazda do tyłu
            HAL_GPIO_WritePin(M1_OUT1_GPIO_Port, M1_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M1_OUT2_GPIO_Port, M1_OUT2_Pin, GPIO_PIN_SET);
        } else { // Hamowanie
            HAL_GPIO_WritePin(M1_OUT1_GPIO_Port, M1_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M1_OUT2_GPIO_Port, M1_OUT2_Pin, GPIO_PIN_RESET);
        }
        // Ustawienie wypełnienia PWM dla silnika M1 (kanał 4)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_duty);
    }
    else if (motor_id == 2) // --- Sterowanie Silnikiem 2 (M2) ---
    {
        if (speed > 0) { // Jazda do przodu
            HAL_GPIO_WritePin(M2_OUT1_GPIO_Port, M2_OUT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M2_OUT2_GPIO_Port, M2_OUT2_Pin, GPIO_PIN_RESET);
        } else if (speed < 0) { // Jazda do tyłu
            HAL_GPIO_WritePin(M2_OUT1_GPIO_Port, M2_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M2_OUT2_GPIO_Port, M2_OUT2_Pin, GPIO_PIN_SET);
        } else { // Hamowanie
            HAL_GPIO_WritePin(M2_OUT1_GPIO_Port, M2_OUT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M2_OUT2_GPIO_Port, M2_OUT2_Pin, GPIO_PIN_RESET);
        }
        // Ustawienie wypełnienia PWM dla silnika M2 (kanał 1)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty);
    }
}

// --- Implementacje funkcji pomocniczych ---

void Motors_Forward(int8_t speed)
{
    Motors_SetSpeed(1, speed);
    Motors_SetSpeed(2, speed);
}

void Motors_Backward(int8_t speed)
{
    Motors_SetSpeed(1, -speed);
    Motors_SetSpeed(2, -speed);
}

void Motors_TurnLeft(int8_t speed)
{
    Motors_SetSpeed(1, -speed);
    Motors_SetSpeed(2, speed);
}

void Motors_TurnRight(int8_t speed)
{
    Motors_SetSpeed(1, speed);
    Motors_SetSpeed(2, -speed);
}

void Motors_Stop(void)
{
    Motors_SetSpeed(1, 0);
    Motors_SetSpeed(2, 0);
}


