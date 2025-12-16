#ifndef INC_SENSORS_TOF_H_
#define INC_SENSORS_TOF_H_

#include "main.h"
#include "vl53l0x_api.h"

// --- Configuration Constants ---
#define TOTAL_SENSOR_COUNT 4
#define TOF_DEFAULT_ADDR   0x52 
#define TOF_BASE_ADDR      0x52 // 0x29 << 1

// --- Data Structures ---

typedef struct {
    GPIO_TypeDef*       xshut_port;
    uint16_t            xshut_pin;
    I2C_HandleTypeDef*  i2c_handle;
} ToF_Config_t;

// --- Public API ---

/**
 * @brief Initializes all sensors defined in the configuration array.
 *        Handles reset, address assignment, calibration, and mode setup.
 */
void ToF_InitAll(const ToF_Config_t* configs, uint8_t count);

/**
 * @brief Reads distance data from all sensors.
 * @param distances Array to store results (size must match TOTAL_SENSOR_COUNT).
 */
void ToF_ReadAll(uint16_t* distances);

/**
 * @brief Reads a single sensor and clears its interrupt.
 */
uint16_t ToF_ReadSingle(uint8_t sensor_index);

/**
 * @brief Clears the interrupt flag for all sensors.
 */
void ToF_ClearAllInterrupts(void);

#endif /* INC_SENSORS_TOF_H_ */