#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

// Uproszczone stany dekodowania protokołu NEC
typedef enum {
    IR_IDLE,       // Oczekiwanie na sygnał startu
    IR_START,      // Odebrano impuls startowy, oczekiwanie na przerwę
    IR_DATA        // Odbieranie bitów danych
} IR_State_t;

// Struktura przechowująca dane i stan dekodera IR
typedef struct {
    uint32_t last_tick;      // Ostatni odczyt licznika timera
    uint8_t bit_count;       // Licznik odebranych bitów
    uint32_t raw_data;       // Surowe, 32-bitowe dane z pilota
    uint8_t address;         // Wyodrębniony adres
    uint8_t command;         // Wyodrębniona komenda
    IR_State_t state;        // Aktualny stan maszyny stanów
    bool data_ready;         // Flaga informująca o odebraniu nowej, poprawnej komendy
} IR_Data_t;

// --- Funkcje publiczne ---

/**
 * @brief Inicjalizuje dekoder IR. Wywołaj raz na początku programu.
 */
void IR_Init(void);

/**
 * @brief Główna funkcja obsługi przerwania. Wywołaj ją ze swojego HAL_GPIO_EXTI_Callback.
 * @note Pin musi mieć skonfigurowane przerwanie na OBA ZBOCZA (Rising and Falling).
 */
void IR_GPIO_Callback(void);

/**
 * @brief Sprawdza, czy odebrano nowe dane z pilota.
 * @return true jeśli dane są gotowe, w przeciwnym razie false.
 */
bool IR_DataReady(void);

/**
 * @brief Zwraca ostatnią poprawnie odebraną komendę.
 * @return Wartość komendy (0-255).
 */
uint8_t IR_GetCommand(void);

/**
 * @brief Czyści flagę gotowości danych. Wywołaj po przetworzeniu komendy.
 */
void IR_ClearFlag(void);

/**
 * @brief Sprawdza, czy robot jest włączony przez pilota.
 * @return true jeśli robot ma działać, w przeciwnym razie false.
 */
bool IR_IsRobotEnabled(void);

/**
 * @brief Przełącza stan robota (włączony/wyłączony).
 */
void IR_ToggleRobot(void);

#endif /* IR_REMOTE_H */