#include "vl53l0x_multi.h"
#include "main.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include <stdio.h>

// Tablica przechowująca stan każdego czujnika
static VL53L0X_Dev_t sensor_devices[TOTAL_SENSOR_COUNT];

// Tablica przechowująca konfigurację każdego czujnika
static const Sensor_Config_t* sensor_configs;

void VL53L0X_Multi_Init(const Sensor_Config_t* configs, uint8_t num_sensors)
{
    sensor_configs = configs;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // Krok 1: Wyłącz wszystkie czujniki
    for (int i = 0; i < num_sensors; i++) {
        HAL_GPIO_WritePin(configs[i].xshut_port, configs[i].xshut_pin, GPIO_PIN_RESET);
    }
    HAL_Delay(20);

    // Krok 2: Włączaj, inicjalizuj i zmień adres każdego czujnika po kolei
    // Liczniki pomogą nam nadać unikalne adresy w obrębie tej samej magistrali
    uint8_t i2c1_sensor_count = 0;
    uint8_t i2c2_sensor_count = 0;

    for (int i = 0; i < num_sensors; i++) {
        // Włącz bieżący czujnik
        HAL_GPIO_WritePin(configs[i].xshut_port, configs[i].xshut_pin, GPIO_PIN_SET);
        HAL_Delay(20);

        // Użyj uchwytu I2C z konfiguracji
        sensor_devices[i].I2cHandle = configs[i].i2c_handle;
        sensor_devices[i].I2cDevAddr = 0x52; // Zawsze startujemy z domyślnego adresu

        // Standardowa inicjalizacja API
        status = VL53L0X_DataInit(&sensor_devices[i]);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();

        status = VL53L0X_StaticInit(&sensor_devices[i]);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();
        
        // Zmień adres I2C, jeśli na tej magistrali jest już inny czujnik
        uint8_t new_address = 0x29; // Domyślny 7-bitowy
        if (configs[i].i2c_handle->Instance == I2C1) {
            new_address += (1 + i2c1_sensor_count); // Adresy od 0x2A dla I2C1
            i2c1_sensor_count++;
        } else if (configs[i].i2c_handle->Instance == I2C2) {
            new_address += (1 + i2c2_sensor_count); // Adresy od 0x2A dla I2C2
            i2c2_sensor_count++;
        }
        
        status = VL53L0X_SetDeviceAddress(&sensor_devices[i], new_address << 1);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();

        sensor_devices[i].I2cDevAddr = new_address << 1;

        // Dokończ inicjalizację
        uint32_t refSpadCount;
        uint8_t isApertureSpads;
        uint8_t VhvSettings;
        uint8_t PhaseCal;
        status = VL53L0X_PerformRefSpadManagement(&sensor_devices[i], &refSpadCount, &isApertureSpads);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();

        status = VL53L0X_PerformRefCalibration(&sensor_devices[i], &VhvSettings, &PhaseCal);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();
        
        // Konfiguracja przerwań dla czujników
        status = VL53L0X_SetGpioConfig(&sensor_devices[i], 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                        VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);
        if(status != VL53L0X_ERROR_NONE){
            Error_Handler();
        }

        status = VL53L0X_SetDeviceMode(&sensor_devices[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();

    }

    // --- FAZA URUCHOMIENIA ---
    for (int i = 0; i < num_sensors; i++) {
        status = VL53L0X_StartMeasurement(&sensor_devices[i]);
        if (status != VL53L0X_ERROR_NONE) Error_Handler();
    }
}

void VL53L0X_Multi_Read(uint16_t* distances)
{
    VL53L0X_RangingMeasurementData_t RangingData;

    for (int i = 0; i < TOTAL_SENSOR_COUNT; i++) {
        VL53L0X_GetRangingMeasurementData(&sensor_devices[i], &RangingData);

        if (RangingData.RangeStatus == 0) {
            distances[i] = RangingData.RangeMilliMeter;
        } else {
            distances[i] = 0xFFFF; // Błąd pomiaru
        }
    }
}


uint16_t VL53L0X_Single_Read(uint8_t sensor_number)
{
    VL53L0X_RangingMeasurementData_t RangingData;

    VL53L0X_GetRangingMeasurementData(&sensor_devices[sensor_number], &RangingData);

    VL53L0X_ClearInterruptMask(&sensor_devices[sensor_number], 0);

    if (RangingData.RangeStatus == 0) {
        return RangingData.RangeMilliMeter;
    } else {
        return 0xFFFF; // Błąd pomiaru
    }
    
}


void VL53L0X_Multi_ClearAllInterrupts(void){

    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    for(int i = 0; i<TOTAL_SENSOR_COUNT; i++){
        status = VL53L0X_ClearInterruptMask(&sensor_devices[i], 0);
        if(status != VL53L0X_ERROR_NONE){
            Error_Handler();
        }
    }
}