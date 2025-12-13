#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

// Stany dekodowania protokołu NEC
typedef enum {
    IR_IDLE,
    IR_START,
    IR_ADDRESS,
    IR_COMMAND,
    IR_REPEAT
} IR_State_t;

// Struktura danych IR
typedef struct {
    uint32_t last_tick;
    uint32_t pulse_width;
    uint8_t bit_count;
    uint32_t raw_data;
    uint8_t address;
    uint8_t command;
    IR_State_t state;
    bool data_ready;
} IR_Data_t;

// Funkcje publiczne
void IR_Init(void);
void IR_GPIO_Callback(void);  // Wywołaj w HAL_GPIO_EXTI_Callback
bool IR_DataReady(void);
uint8_t IR_GetCommand(void);
void IR_ClearFlag(void);
bool IR_IsRobotEnabled(void);  // Sprawdź czy robot włączony
void IR_ToggleRobot(void);     // Przełącz stan robota

#endif /* IR_REMOTE_H */