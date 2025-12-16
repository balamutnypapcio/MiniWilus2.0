/*
 * sensors_tof.c
 */

#include "tof_sensors.h"
#include "vl53l0x_def.h"

// --- Private Variables ---

static VL53L0X_Dev_t sensor_devices[TOTAL_SENSOR_COUNT];
static const ToF_Config_t* sensor_configs;

// --- Private Helper Macros ---

// simplistic error check to keep code clean
#define CHECK_STATUS(status) if ((status) != VL53L0X_ERROR_NONE) Error_Handler();

// --- Private Function Prototypes ---

static void DisableAllSensors(uint8_t count);
static void EnableSensor(uint8_t index);
static void InitializeSensorDriver(uint8_t index);
static void AssignUniqueAddress(uint8_t index, uint8_t* i2c1_cnt, uint8_t* i2c2_cnt);
static void CalibrateSensor(uint8_t index);
static void ConfigureInterruptAndStart(uint8_t index);

// --- Public Function Implementation ---

void ToF_InitAll(const ToF_Config_t* configs, uint8_t count)
{
    sensor_configs = configs;
    
    DisableAllSensors(count);
    
    // Counters to track devices on separate I2C buses for addressing
    uint8_t i2c1_sensor_count = 0;
    uint8_t i2c2_sensor_count = 0;

    // 2. Sequential Initialization Loop
    for (uint8_t i = 0; i < count; i++) 
    {
        EnableSensor(i);
        InitializeSensorDriver(i);
        AssignUniqueAddress(i, &i2c1_sensor_count, &i2c2_sensor_count);
        CalibrateSensor(i);
        ConfigureInterruptAndStart(i);
    }
}

void ToF_ReadAll(uint16_t* distances)
{
    VL53L0X_RangingMeasurementData_t rangingData;
    for (int i = 0; i < TOTAL_SENSOR_COUNT; i++) 
    {
        VL53L0X_GetRangingMeasurementData(&sensor_devices[i], &rangingData);
        if (rangingData.RangeStatus == 0) {
            distances[i] = rangingData.RangeMilliMeter;
        } else {
            distances[i] = 0xFFFF; // Error value
        }
    }
}

uint16_t ToF_ReadSingle(uint8_t sensor_index)
{
    if (sensor_index >= TOTAL_SENSOR_COUNT) return 0xFFFF;
    VL53L0X_RangingMeasurementData_t rangingData;
    VL53L0X_GetRangingMeasurementData(&sensor_devices[sensor_index], &rangingData);
    
    // Clear interrupt to allow next measurement
    VL53L0X_ClearInterruptMask(&sensor_devices[sensor_index], 0);

    if (rangingData.RangeStatus == 0) {
        return rangingData.RangeMilliMeter;
    }
    return 0xFFFF;
}

void ToF_ClearAllInterrupts(void)
{
    for(int i = 0; i < TOTAL_SENSOR_COUNT; i++) {
        VL53L0X_ClearInterruptMask(&sensor_devices[i], 0);
    }
}

// --- Private Function Implementation ---

static void DisableAllSensors(uint8_t count)
{
    for (int i = 0; i < count; i++) {
        HAL_GPIO_WritePin(sensor_configs[i].xshut_port, sensor_configs[i].xshut_pin, GPIO_PIN_RESET);
    }
    HAL_Delay(20); // Hardware boot time margin
}

static void EnableSensor(uint8_t index)
{
    HAL_GPIO_WritePin(sensor_configs[index].xshut_port, sensor_configs[index].xshut_pin, GPIO_PIN_SET);
    HAL_Delay(20); // Hardware boot time margin
}

static void InitializeSensorDriver(uint8_t index)
{
    // Assign I2C handle from config
    sensor_devices[index].I2cHandle = sensor_configs[index].i2c_handle;
    sensor_devices[index].I2cDevAddr = TOF_DEFAULT_ADDR;

    VL53L0X_Error status;
    status = VL53L0X_DataInit(&sensor_devices[index]);
    CHECK_STATUS(status);

    status = VL53L0X_StaticInit(&sensor_devices[index]);
    CHECK_STATUS(status);
}

static void AssignUniqueAddress(uint8_t index, uint8_t* i2c1_cnt, uint8_t* i2c2_cnt)
{
    uint8_t new_address_7bit = 0x29 + 1; 

    if (sensor_devices[index].I2cHandle->Instance == I2C1) {
        new_address_7bit += (*i2c1_cnt);
        (*i2c1_cnt)++;
    } 
    else if (sensor_devices[index].I2cHandle->Instance == I2C2) {
        new_address_7bit += (*i2c2_cnt);
        (*i2c2_cnt)++;
    }

    // Convert to 8-bit address for the driver
    uint8_t final_address = new_address_7bit << 1;

    VL53L0X_Error status = VL53L0X_SetDeviceAddress(&sensor_devices[index], final_address);
    CHECK_STATUS(status);

    // Update the device structure with the new address
    sensor_devices[index].I2cDevAddr = final_address;
}

static void CalibrateSensor(uint8_t index)
{
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t vhvSettings;
    uint8_t phaseCal;
    VL53L0X_Error status;

    status = VL53L0X_PerformRefSpadManagement(&sensor_devices[index], &refSpadCount, &isApertureSpads);
    CHECK_STATUS(status);

    status = VL53L0X_PerformRefCalibration(&sensor_devices[index], &vhvSettings, &phaseCal);
    CHECK_STATUS(status);
}

static void ConfigureInterruptAndStart(uint8_t index)
{
    VL53L0X_Error status;

    // Enable GPIO1 interrupt when new measurement is ready (Active Low)
    status = VL53L0X_SetGpioConfig(&sensor_devices[index], 0, 
                                   VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                                   VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, 
                                   VL53L0X_INTERRUPTPOLARITY_LOW);
    CHECK_STATUS(status);

    status = VL53L0X_SetDeviceMode(&sensor_devices[index], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    CHECK_STATUS(status);

    status = VL53L0X_StartMeasurement(&sensor_devices[index]);
    CHECK_STATUS(status);
}