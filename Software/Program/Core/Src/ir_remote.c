#include "ir_remote.h"
#include "tim.h"      // Musi zawierać definicję Twojego timera
#include "main.h"     // Musi zawierać definicje GPIO (np. STARTER_GPIO_Port)

// ##################################################################################
// # KONFIGURACJA - DOSTOSUJ DO SWOJEGO PROJEKTU                                    #
// ##################################################################################

// ZMIEŃ NA UCHWYT DO TIMERA, KTÓREGO UŻYWASZ DO POMIARU CZASU
// Timer musi być skonfigurowany:
// - Prescaler: tak, aby 1 tick = 1µs (np. 169 dla zegara 170MHz)
// - Counter Period (ARR): 65535 (maksymalna wartość)
// - Musi być uruchomiony w main(): HAL_TIM_Base_Start(&htim_ir);
extern TIM_HandleTypeDef htim2; 
#define htim_ir htim2

// ZMIEŃ NA DEFINICJE TWOJEGO PINU ODBIORNIKA IR
// Pin musi mieć włączone przerwanie EXTI na OBA ZBOCZA (Rising and Falling edge)
#define IR_GPIO_PORT  STARTER_GPIO_Port
#define IR_GPIO_PIN   STARTER_Pin

// ##################################################################################


// Timings dla protokołu NEC (w mikrosekundach)
#define IR_START_MIN    8000  // Impuls startowy ~9ms
#define IR_START_MAX    10000
#define IR_SPACE_MIN    4000  // Przerwa po starcie ~4.5ms
#define IR_SPACE_MAX    5000
#define IR_REPEAT_MIN   2000  // Przerwa dla kodu powtórzenia ~2.25ms
#define IR_REPEAT_MAX   2500
#define IR_BIT_MIN      300   // Przerwa dla bitu '0' ~560µs
#define IR_BIT_MAX      800
#define IR_ONE_MIN      1400  // Przerwa dla bitu '1' ~1690µs
#define IR_ONE_MAX      1900
#define IR_TIMEOUT      12000 // Timeout w µs (~12ms). Dłuższa cisza resetuje maszynę stanów.

// Zmienne statyczne (prywatne dla tego pliku)
static IR_Data_t ir_data;
static volatile bool robot_enabled = false;
// ... na górze pliku, z innymi zmiennymi ...
volatile uint32_t debug_timings[100];
volatile int debug_idx = 0;

// Prototyp funkcji prywatnej
static void IR_ProcessEdge(uint32_t delta, GPIO_PinState pin_after_edge);


// --- Implementacja funkcji publicznych ---

void IR_Init(void) {
    ir_data.state = IR_IDLE;
    ir_data.bit_count = 0;
    ir_data.raw_data = 0;
    ir_data.data_ready = false;
    // Ustawia początkowy tick, aby uniknąć błędnego pomiaru przy pierwszym zboczu
    ir_data.last_tick = __HAL_TIM_GET_COUNTER(&htim_ir);
    robot_enabled = false;
}

bool IR_IsRobotEnabled(void) {
    return robot_enabled;
}

void IR_ToggleRobot(void) {
    static uint32_t last_toggle_time = 0;
    uint32_t now = HAL_GetTick();
    
    // Prosty debouncing, aby uniknąć wielokrotnego przełączenia od jednego naciśnięcia
    if (now - last_toggle_time < 500) {
        return;
    }
    
    robot_enabled = !robot_enabled;
    last_toggle_time = now;
}

void IR_GPIO_Callback(void) {
    // 1. Złap czas NATYCHMIAST, aby pomiar był jak najdokładniejszy
    uint32_t current_tick = __HAL_TIM_GET_COUNTER(&htim_ir);
    
    // 2. Oblicz różnicę czasu (z obsługą przepełnienia licznika)
    uint32_t delta;
    if (current_tick >= ir_data.last_tick) {
        delta = current_tick - ir_data.last_tick;
    } else {
        // Zaszło przepełnienie licznika (zakładając, że ARR = 65535)
        delta = (65535 - ir_data.last_tick) + current_tick;
    }
    
    // 3. Zaktualizuj ostatni czas
    ir_data.last_tick = current_tick;
    
    // 4. Sprawdź stan pinu PO wystąpieniu zbocza
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_GPIO_PIN);
    
    // 5. Przekaż dane do przetworzenia
    IR_ProcessEdge(delta, pin_state);
}

bool IR_DataReady(void) {
    return ir_data.data_ready;
}

uint8_t IR_GetCommand(void) {
    return ir_data.command;
}

void IR_ClearFlag(void) {
    ir_data.data_ready = false;
}


// --- Implementacja funkcji prywatnych ---

/**
 * @brief Główna maszyna stanów dekodera NEC.
 * @param delta Czas trwania ostatniego impulsu/przerwy w mikrosekundach.
 * @param pin_after_edge Stan pinu PO zboczu (SET = koniec impulsu, RESET = koniec przerwy).
 */
static void IR_ProcessEdge(uint32_t delta, GPIO_PinState pin_after_edge) {

    if (debug_idx < 100) {
        debug_timings[debug_idx++] = delta;
    }


    if (delta > IR_TIMEOUT) {
        ir_data.state = IR_IDLE;
        return;
    }

    switch (ir_data.state) {
        case IR_IDLE:
            // Oczekujemy na koniec impulsu startowego (~9ms).
            // Zbocze musi być narastające (koniec stanu niskiego).
            if (pin_after_edge == GPIO_PIN_SET && delta > IR_START_MIN && delta < IR_START_MAX) {
                ir_data.state = IR_START;
                ir_data.bit_count = 0;
                ir_data.raw_data = 0;
            }
            break;
            
        case IR_START:
            // Oczekujemy na koniec przerwy po starcie.
            // Zbocze musi być opadające (koniec stanu wysokiego).
            if (pin_after_edge == GPIO_PIN_RESET) {
                if (delta > IR_SPACE_MIN && delta < IR_SPACE_MAX) {
                    ir_data.state = IR_DATA; // Poprawna przerwa, zacznij odbierać dane
                } else if (delta > IR_REPEAT_MIN && delta < IR_REPEAT_MAX) {
                    // Odebrano kod powtórzenia, zignoruj i wróć do IDLE
                    // (można dodać logikę obsługi przytrzymania przycisku)
                    ir_data.state = IR_IDLE;
                } else {
                    ir_data.state = IR_IDLE; // Błędny timing, reset
                }
            }
            break;
            
        case IR_DATA:
            // W stanie danych informacja jest zakodowana w długości PRZERWY (stan wysoki).
            // Dlatego reagujemy tylko na zbocze opadające.
            if (pin_after_edge == GPIO_PIN_RESET) {
                ir_data.raw_data >>= 1;
                
                if (delta > IR_ONE_MIN && delta < IR_ONE_MAX) { // Długa przerwa to '1'
                    ir_data.raw_data |= 0x80000000;
                } else if (delta > IR_BIT_MIN && delta < IR_BIT_MAX) { // Krótka przerwa to '0'
                    // Nie robimy nic, bo przesunięcie bitu w prawo już wstawiło zero.
                } else {
                    ir_data.state = IR_IDLE; // Błąd timingu
                    break;
                }
                
                ir_data.bit_count++;
                if (ir_data.bit_count >= 32) {
                    // Odebrano wszystkie 32 bity, czas na walidację
                    uint8_t address = (ir_data.raw_data >> 24) & 0xFF;
                    uint8_t address_inv = (ir_data.raw_data >> 16) & 0xFF;
                    uint8_t command = (ir_data.raw_data >> 8) & 0xFF;
                    uint8_t command_inv = ir_data.raw_data & 0xFF;

                    // Sprawdź poprawność danych (adres vs ~adres, komenda vs ~komenda)
                    if ((uint8_t)(address + address_inv) == 0xFF && (uint8_t)(command + command_inv) == 0xFF) {
                        ir_data.address = address;
                        ir_data.command = command;
                        ir_data.data_ready = true; // Ustaw flagę, że dane są gotowe
                        
                        // Wykonaj akcję przypisaną do przycisku
                        IR_ToggleRobot();
                    }
                    
                    ir_data.state = IR_IDLE; // Resetuj do stanu początkowego
                }
            }
            // Ignorujemy zbocza narastające w stanie IR_DATA, ponieważ długość impulsu (~560µs)
            // w protokole NEC jest zawsze taka sama i nie niesie informacji o bicie.
            break;
    }
}