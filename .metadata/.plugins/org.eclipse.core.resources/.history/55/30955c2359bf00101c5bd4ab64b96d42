/*
 * timing.c
 *
 *  Created on: Nov 11, 2025
 *      Author: jakub
 */


#include "timing.h"
#include "tim.h" // Potrzebujemy uchwytu htim6

// Ta funkcja jest wywoływana z bibliotek HAL. Musimy ją tu zdefiniować,
// aby linker podmienił domyślną, niedziałającą wersję.
// Atrybut "__weak" jest ważny, aby uniknąć konfliktów.
__weak void HAL_Delay(uint32_t Delay)
{
  MyDelay_ms(Delay);
}

// Nasza niezawodna implementacja opóźnienia
void MyDelay_ms(uint16_t ms)
{
  for (uint16_t i = 0; i < ms; i++)
  {
    __HAL_TIM_SET_COUNTER(&htim6, 0);  // Wyzeruj licznik
    while (__HAL_TIM_GET_COUNTER(&htim6) < 10); // Czekaj na 10 ticków (1 ms)
  }
}
