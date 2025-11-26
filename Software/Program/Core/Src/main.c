/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "motors.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "vl53l0x_api.h"
#include "vl53l0x_multi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENEMY_DETECT_DISTANCE 200
#define ENEMY_FAR_DISTANCE 500
#define LINE_THRESHOLD 2000
#define TRACKING_TIMEOUT 800
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Stany robota
typedef enum {
    STATE_SEARCH,
    STATE_ATTACK,
    STATE_TRACKING,
    STATE_ESCAPE_LINE,
    STATE_TURN_SEARCH
} RobotState;

// Ostatni znany kierunek przeciwnika
typedef enum {
    DIR_UNKNOWN,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_FRONT
} LastDirection;

typedef struct {
    bool front_right;
    bool right;
    bool left;
    bool front_left;
    bool any_detected;
} EnemyDetection;


Sensor_Config_t sensor_configs[TOTAL_SENSOR_COUNT] = {
    { XSHUT1_GPIO_Port, XSHUT1_Pin, &hi2c1 },  // Prawy przedni
    { XSHUT2_GPIO_Port, XSHUT2_Pin, &hi2c1 },  // Prawy
    { XSHUT3_GPIO_Port, XSHUT3_Pin, &hi2c2 },  // Lewy
    { XSHUT4_GPIO_Port, XSHUT4_Pin, &hi2c2 }   // Lewy przedni
};

volatile bool g_sensor1_data_ready = false;
volatile bool g_sensor2_data_ready = false;
volatile bool g_sensor3_data_ready = false;
volatile bool g_sensor4_data_ready = false;
uint16_t sensor_distances[4];

volatile uint16_t adc_values[3];
RobotState current_state = STATE_SEARCH;
LastDirection last_enemy_direction = DIR_UNKNOWN;
uint32_t last_detection_time = 0;
uint32_t state_start_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcja sprawdzająca wykrycie linii
bool IsOnLine(void) {
    return (adc_values[0] > LINE_THRESHOLD || 
            adc_values[1] > LINE_THRESHOLD || 
            adc_values[2] > LINE_THRESHOLD);
}


// Funkcja aktualizująca detekcję przeciwnika
void UpdateEnemyDetection(EnemyDetection *enemy) {
    enemy->front_right = (sensor_distances[0] < ENEMY_DETECT_DISTANCE);
    enemy->right = (sensor_distances[1] < ENEMY_DETECT_DISTANCE);
    enemy->left = (sensor_distances[2] < ENEMY_DETECT_DISTANCE);
    enemy->front_left = (sensor_distances[3] < ENEMY_DETECT_DISTANCE);
    
    enemy->any_detected = enemy->front_right || enemy->right || 
                          enemy->left || enemy->front_left;
}

void UpdateLastDirection(EnemyDetection *enemy) {
    if (!enemy->any_detected) {
        return;  // Nie aktualizuj jeśli nic nie widzimy
    }
    
    last_detection_time = HAL_GetTick();
    
    // Ustal kierunek na podstawie czujników
    if (enemy->front_left && enemy->front_right) {
        last_enemy_direction = DIR_FRONT;
    }
    else if (enemy->left || enemy->front_left) {
        last_enemy_direction = DIR_LEFT;
    }
    else if (enemy->right || enemy->front_right) {
        last_enemy_direction = DIR_RIGHT;
    }
}


void EscapeLine(){
    Motors_Backward(100);
    HAL_Delay(300);
    
    if (adc_values[0] > LINE_THRESHOLD) {
        Motors_TurnLeft(100);
    } else if (adc_values[2] > LINE_THRESHOLD) {
        Motors_TurnRight(100);
    } else {
        Motors_TurnLeft(100);
    }
    HAL_Delay(200);
}


void RobotControl(EnemyDetection *enemy) {
    uint32_t current_time = HAL_GetTick();
    
    // // Najwyższy priorytet: linia
    if (IsOnLine()) {
        current_state = STATE_ESCAPE_LINE;
        state_start_time = current_time;
        
        //EscapeLine();
        
        current_state = STATE_SEARCH;
        last_enemy_direction = DIR_UNKNOWN;  // Resetuj kierunek
        return;
    }
    
    // Maszyna stanów
    switch (current_state) {
        case STATE_SEARCH:
            if (enemy->any_detected) {
                UpdateLastDirection(enemy);
                current_state = STATE_ATTACK;
                state_start_time = current_time;
            } else {
                // Szukaj przeciwnika - powolny obrót
                Motors_TurnRight(50/1);
            }
            break;
            
        case STATE_ATTACK:
            if (enemy->any_detected) {
                UpdateLastDirection(enemy);  // Ciągle aktualizuj kierunek
                
                // Priorytet: atak frontalny
                if (enemy->front_left && enemy->front_right) {
                    // Przeciwnik DOKŁADNIE z przodu - full speed!
                    Motors_Forward(100/1);
                }
                else if (enemy->front_left && !enemy->front_right) {
                    // Lekko z lewej - skręcaj powoli w lewo jadąc do przodu
                    Motors_SetSpeed(1, 80/1);  // Lewy wolniej
                    Motors_SetSpeed(2, 100/1); // Prawy szybciej
                }
                else if (enemy->front_right && !enemy->front_left) {
                    // Lekko z prawej - skręcaj powoli w prawo jadąc do przodu
                    Motors_SetSpeed(1, 100/1);  // Lewy szybciej
                    Motors_SetSpeed(2, 80/1);  // Prawy wolniej
                }
                else if (enemy->left && !enemy->front_left) {
                    // Tylko lewy czujnik (z boku) - ostry obrót w lewo
                    Motors_TurnLeft(90/1);
                }
                else if (enemy->right && !enemy->front_right) {
                    // Tylko prawy czujnik (z boku) - ostry obrót w prawo
                    Motors_TurnRight(90/1);
                }
            }
            else {
                // UTRACONO KONTAKT - przejdź do śledzenia
                current_state = STATE_TRACKING;
                state_start_time = current_time;
            }
            break;
            
        case STATE_TRACKING:
            // MARTWE POLE - kontynuuj obrót w ostatnim znanym kierunku
            
            if (enemy->any_detected) {
                // ZNALEZIONO PONOWNIE!
                UpdateLastDirection(enemy);
                current_state = STATE_ATTACK;
                state_start_time = current_time;
                break;
            }
            
            // Sprawdź timeout
            if (current_time - last_detection_time > TRACKING_TIMEOUT) {
                // Za długo w martwym polu - wróć do szukania
                current_state = STATE_SEARCH;
                last_enemy_direction = DIR_UNKNOWN;
                break;
            }
            
            // Kontynuuj obrót w ostatnim znanym kierunku
            switch (last_enemy_direction) {
                case DIR_LEFT:
                    // Był z lewej, więc kręć w lewo aż go znowu zobaczysz
                    Motors_TurnLeft(80/1);
                    break;
                    
                case DIR_RIGHT:
                    // Był z prawej, więc kręć w prawo
                    Motors_TurnRight(80/1);
                    break;
                    
                case DIR_FRONT:
                    // Był z przodu - jedź do przodu i lekko w lewo/prawo
                    // (możesz dodać logikę która strona była ostatnia)
                    Motors_Forward(70/1);
                    break;
                    
                default:
                    // Nie wiadomo gdzie był - wróć do szukania
                    current_state = STATE_SEARCH;
                    break;
            }
            break;
            
        case STATE_ESCAPE_LINE:
            current_state = STATE_SEARCH;
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  VL53L0X_Multi_Init(sensor_configs, TOTAL_SENSOR_COUNT);

  g_sensor1_data_ready = false;
  g_sensor2_data_ready = false;
  g_sensor3_data_ready = false;
  g_sensor4_data_ready = false;
  VL53L0X_Multi_ClearAllInterrupts();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 3);

  EnemyDetection enemy = {0};
  Motors_Init();

  HAL_Delay(10000);
  for(int i = 0; i<3; i++){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
  HAL_Delay(300);
  }

  HAL_Delay(5000);
  for(int i = 0; i<2; i++){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
  HAL_Delay(300);
  }

  HAL_Delay(5000);
  for(int i = 0; i<1; i++){
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  HAL_Delay(300);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
  HAL_Delay(300);
  }

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(g_sensor1_data_ready){
      g_sensor1_data_ready = false;
      sensor_distances[0] = VL53L0X_Single_Read(0);
    }
    if(g_sensor2_data_ready){
      g_sensor2_data_ready = false;
      sensor_distances[1] = VL53L0X_Single_Read(1);
    }
    if(g_sensor3_data_ready){
      g_sensor3_data_ready = false;
      sensor_distances[2] = VL53L0X_Single_Read(2);
    }
    if(g_sensor4_data_ready){
      g_sensor4_data_ready = false;
      sensor_distances[3] = VL53L0X_Single_Read(3);
    }

    //LED diagnostyczne
    if(enemy.any_detected) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
    }   

    // Aktualizuj detekcję przeciwnika
    UpdateEnemyDetection(&enemy);
        
    // Wykonaj logikę sterowania
    RobotControl(&enemy);
        
    // Krótkie opóźnienie dla stabilności
    //HAL_Delay(10);
    //HAL_Delay(1000);
    //uint16_t current_dist = VL53L0X_Single_Read(0); 
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch (GPIO_Pin)
  {
    case EXTI1_Pin:
      g_sensor1_data_ready = true;
      break;
    case EXTI2_Pin:
      g_sensor2_data_ready = true;
      break;
    case EXTI3_Pin:
      g_sensor3_data_ready = true;
      break;
    case EXTI4_Pin:
      g_sensor4_data_ready = true;
      break;
    default:
      break;
  }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
