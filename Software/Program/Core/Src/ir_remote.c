#include "ir_remote.h"
#include "tim.h"

// Timings dla protokołu NEC (w mikrosekundach)
#define IR_START_MIN    9000    // Start burst: obserwowane 9791-15489
#define IR_START_MAX    17000
#define IR_SPACE_MIN    11000   // Space: obserwowane 13735-13754
#define IR_SPACE_MAX    15000
#define IR_BIT_MIN      800     // Logical 0: ~1150
#define IR_BIT_MAX      1400
#define IR_ONE_MIN      1900    // Logical 1: ~2250
#define IR_ONE_MAX      2600
#define IR_REPEAT_MIN   3000
#define IR_REPEAT_MAX   4000
#define IR_TIMEOUT      200000  // Zwiększony timeout do 200ms

static IR_Data_t ir_data;
static volatile bool robot_enabled = false;  // Czy robot ma działać
extern TIM_HandleTypeDef htim2;  

// Inicjalizacja
void IR_Init(void) {
    ir_data.state = IR_IDLE;
    ir_data.bit_count = 0;
    ir_data.raw_data = 0;
    ir_data.data_ready = false;
    ir_data.last_tick = __HAL_TIM_GET_COUNTER(&htim2); // ← DODAJ - ustaw początkowy tick
    robot_enabled = false;
}

// Funkcja sprawdzająca czy robot jest włączony
bool IR_IsRobotEnabled(void) {
    return robot_enabled;
}

// Toggle stanu robota (włącz/wyłącz)
void IR_ToggleRobot(void) {
    robot_enabled = !robot_enabled;
}


// Sprawdź czy są nowe dane
bool IR_DataReady(void) {
    return ir_data.data_ready;
}

// Pobierz ostatnią komendę
uint8_t IR_GetCommand(void) {
    return ir_data.command;
}

// Wyczyść flagę danych
void IR_ClearFlag(void) {
    ir_data.data_ready = false;
}
