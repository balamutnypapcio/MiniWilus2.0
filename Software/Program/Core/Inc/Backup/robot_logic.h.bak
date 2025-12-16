#ifndef INC_ROBOT_LOGIC_H_
#define INC_ROBOT_LOGIC_H_

#include <stdbool.h>
#include <stdint.h>

// --- Configuration Constants ---
#define LINE_THRESHOLD        230  // ADC value threshold for line detection
#define ENEMY_DIST_CLOSE      200   // Distance in mm for immediate attack
#define ENEMY_DIST_FAR        500   // Max detection distance
#define TIME_TRACKING_LOST    800   // Time in ms before giving up tracking

// --- Speed Constants (Percent) ---
#define SPEED_MAX             100
#define SPEED_ATTACK_TURN     90
#define SPEED_SEARCH          100
#define SPEED_TRACKING        80
#define SPEED_ESCAPE          100

// --- Data Types ---

typedef enum {
    STATE_SEARCH,
    STATE_ATTACK,
    STATE_TRACKING,
    STATE_ESCAPE_LINE
} RobotState_t;

// Structure to simplify sensor data analysis
typedef struct {
    bool front_right;
    bool right;
    bool left;
    bool front_left;
    bool any_detected;
} EnemyData_t;

typedef enum {
    LINE_POS_RIGHT = 0, // LS1 (ADC buffer index 0)
    LINE_POS_BACK  = 1, // LS2 (ADC buffer index 1)
    LINE_POS_LEFT  = 2  // LS3 (ADC buffer index 2)
} LineSensorPos_t;


// --- Public API ---

/**
 * @brief Initializes the logic module (sets initial state).
 */
void Logic_Init(void);

/**
 * @brief Main logic loop. Should be called periodically in the main loop.
 * @param line_sensors Pointer to array of 3 raw ADC values (Left, Right, Back/Middle).
 * @param dist_sensors Pointer to array of 4 distance values in mm.
 */
void Logic_Update(uint16_t* line_sensors, uint16_t* dist_sensors);

#endif /* INC_ROBOT_LOGIC_H_ */