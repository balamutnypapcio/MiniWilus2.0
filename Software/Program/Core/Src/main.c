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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Sensor_Config_t sensor_configs[TOTAL_SENSOR_COUNT] = {
    // Czujnik 0: na I2C1, sterowany przez XSHUT1
    { XSHUT1_GPIO_Port, XSHUT1_Pin, &hi2c1 },
    
    // Czujnik 1: na I2C1, sterowany przez XSHUT2
    { XSHUT2_GPIO_Port, XSHUT2_Pin, &hi2c1 },

    // Czujnik 2: na I2C2, sterowany przez XSHUT3
    { XSHUT3_GPIO_Port, XSHUT3_Pin, &hi2c2 },

    // Czujnik 3: na I2C2, sterowany przez XSHUT4
    { XSHUT4_GPIO_Port, XSHUT4_Pin, &hi2c2 }
};

volatile bool g_sensor1_data_ready = false;
volatile bool g_sensor2_data_ready = false;
volatile bool g_sensor3_data_ready = false;
volatile bool g_sensor4_data_ready = false;
uint16_t sensor_distances[4];

volatile uint16_t adc_values[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  uint16_t ls1_value; // Wynik z PA0 (Rank 1)
  uint16_t ls2_value; // Wynik z PA1 (Rank 2)
  uint16_t ls3_value; // Wynik z PA2 (Rank 3)

  Motors_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //HAL_Delay(500);
    /* USER CODE END WHILE */
    Motors_Forward(30);
    /* USER CODE BEGIN 3 */

    ls1_value = adc_values[0]; // Wynik z PA0 (Rank 1)
    ls2_value = adc_values[1]; // Wynik z PA1 (Rank 2)
    ls3_value = adc_values[2]; // Wynik z PA2 (Rank 3)

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

    if((sensor_distances[0] < 800 && sensor_distances[0] > 30) || 
       (sensor_distances[1] < 800 && sensor_distances[1] > 30) ||
       (sensor_distances[2] < 800 && sensor_distances[2] > 30) || 
       (sensor_distances[3] < 800 && sensor_distances[3] > 30)){
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
    } else {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
    }
    //HAL_Delay(1000);
    //uint16_t current_dist = VL53L0X_Single_Read(0); 
    
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
  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  switch (GPIO_Pin)
  {
    case EXTI1_Pin: // Użyj etykiety z CubeMX dla pinu czujnika 1 (np. PC13)
      g_sensor1_data_ready = true;
      break;

    case EXTI2_Pin: // Użyj etykiety z CubeMX dla pinu czujnika 2 (np. PC14)
      g_sensor2_data_ready = true;
      break;

    case EXTI3_Pin: // Użyj etykiety z CubeMX dla pinu czujnika 1 (np. PC13)
      g_sensor3_data_ready = true;
      break;

    case EXTI4_Pin: // Użyj etykiety z CubeMX dla pinu czujnika 2 (np. PC14)
      g_sensor4_data_ready = true;
      break;

    default:
      // Przerwanie z innego pinu, ignoruj
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
