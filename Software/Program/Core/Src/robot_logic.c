#include "robot_logic.h"
#include "motors.h"
#include "main.h" // Required for HAL_Delay and HAL_GetTick

// --- Private Variables ---
static RobotState_t current_state = STATE_SEARCH;
static uint32_t last_detection_time = 0;

// Internal enum to remember the last position of the opponent
typedef enum {
    DIR_UNKNOWN,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_FRONT
} EnemyDirection_t;

static EnemyDirection_t last_enemy_dir = DIR_UNKNOWN;

// --- Private Helper Functions ---

static bool isLineDetected(uint16_t* line_sensors) {
    bool right = line_sensors[LINE_POS_RIGHT] > LINE_THRESHOLD;
    bool back  = line_sensors[LINE_POS_BACK]  > LINE_THRESHOLD;
    bool left  = line_sensors[LINE_POS_LEFT]  > LINE_THRESHOLD;

    return (right || back || left);
}


static void updateEnemyData(EnemyData_t *enemy, uint16_t* distances) {
    enemy->front_right = (distances[0] < ENEMY_DIST_CLOSE);
    enemy->right       = (distances[1] < ENEMY_DIST_CLOSE);
    enemy->left        = (distances[2] < ENEMY_DIST_CLOSE);
    enemy->front_left  = (distances[3] < ENEMY_DIST_CLOSE);
    
    enemy->any_detected = enemy->front_right || enemy->right || 
                          enemy->left || enemy->front_left;
}

static void updateTrackingDirection(EnemyData_t *enemy) {
    if (!enemy->any_detected) return;
    
    last_detection_time = HAL_GetTick();
    
    if (enemy->front_left && enemy->front_right) {
        last_enemy_dir = DIR_FRONT;
    } else if (enemy->left || enemy->front_left) {
        last_enemy_dir = DIR_LEFT;
    } else if (enemy->right || enemy->front_right) {
        last_enemy_dir = DIR_RIGHT;
    }
}

// --- State Handlers ---

static void handleStateSearch(EnemyData_t *enemy) {
    if (enemy->any_detected) {
        updateTrackingDirection(enemy);
        current_state = STATE_ATTACK;
    } else {
        Motors_TurnRight(SPEED_SEARCH);
    }
}


static void handleStateEscapeLine(uint16_t* line_sensors) {
    
    // Check which specific sensor sees the line
    bool see_right = line_sensors[LINE_POS_RIGHT] > LINE_THRESHOLD;
    bool see_left  = line_sensors[LINE_POS_LEFT]  > LINE_THRESHOLD;
    bool see_back  = line_sensors[LINE_POS_BACK]  > LINE_THRESHOLD;

    // Intelligent reaction
    if (see_back) {
        Motors_Forward(SPEED_ESCAPE);
    } 
    else if (see_left) {
        Motors_Backward(SPEED_ESCAPE);
        HAL_Delay(50); 
        //Motors_TurnRight(SPEED_ESCAPE);
    } 
    else if (see_right) {
        Motors_Backward(SPEED_ESCAPE);
        HAL_Delay(50); // Short reverse
        //Motors_TurnLeft(SPEED_ESCAPE);
    } 
    else {
        // Fallback (e.g. noise) -> Just back off
        Motors_Backward(SPEED_ESCAPE);
    }

    // Blocking delay for the maneuver (in simple FSM)
    // For advanced FSM, use timers instead of HAL_Delay
    //HAL_Delay(200); 
    
    current_state = STATE_SEARCH;
    last_enemy_dir = DIR_UNKNOWN;
}

static void handleStateAttack(EnemyData_t *enemy) {
    if (!enemy->any_detected) {
        current_state = STATE_TRACKING;
        return;
    }

    updateTrackingDirection(enemy);

    // Attack Logic Priority: Front -> Front-Side -> Side
    if (enemy->front_left && enemy->front_right) {
        // Opponent is dead ahead - Full speed!
        Motors_Forward(SPEED_MAX);
    } 
    else if (enemy->front_left) {
        // Slightly to the left - Turn left while moving forward
        Motors_SetSpeed(MOTOR_LEFT, SPEED_TRACKING);
        Motors_SetSpeed(MOTOR_RIGHT, SPEED_MAX);
    } 
    else if (enemy->front_right) {
        // Slightly to the right - Turn right while moving forward
        Motors_SetSpeed(MOTOR_LEFT, SPEED_MAX);
        Motors_SetSpeed(MOTOR_RIGHT, SPEED_TRACKING);
    } 
    else if (enemy->left) {
        // Sharp turn left
        Motors_TurnLeft(SPEED_ATTACK_TURN);
    } 
    else if (enemy->right) {
        // Sharp turn right
        Motors_TurnRight(SPEED_ATTACK_TURN);
    }
}

static void handleStateTracking(EnemyData_t *enemy) {
    if (enemy->any_detected) {
        updateTrackingDirection(enemy);
        current_state = STATE_ATTACK;
        return;
    }

    // Check timeout - if lost for too long, go back to Search
    if (HAL_GetTick() - last_detection_time > TIME_TRACKING_LOST) {
        current_state = STATE_SEARCH;
        last_enemy_dir = DIR_UNKNOWN;
        return;
    }

    // Spin towards the last known position
    switch (last_enemy_dir) {
        case DIR_LEFT:  Motors_TurnLeft(SPEED_TRACKING); break;
        case DIR_RIGHT: Motors_TurnRight(SPEED_TRACKING); break;
        case DIR_FRONT: Motors_Forward(SPEED_TRACKING); break; // Or random turn
        default:        current_state = STATE_SEARCH; break;
    }
}
// --- Public Function Implementation ---

void Logic_Init(void) {
    current_state = STATE_SEARCH;
    last_enemy_dir = DIR_UNKNOWN;
}

void Logic_Update(uint16_t* line_sensors, uint16_t* dist_sensors) {
    
    if (isLineDetected(line_sensors)) {
        current_state = STATE_ESCAPE_LINE;
    }
    EnemyData_t enemy = {0};
    updateEnemyData(&enemy, dist_sensors);

    switch (current_state) {
        case STATE_ESCAPE_LINE: handleStateEscapeLine(line_sensors); break;
        case STATE_SEARCH:      handleStateSearch(&enemy); break;
        case STATE_ATTACK:      handleStateAttack(&enemy); break;
        case STATE_TRACKING:    handleStateTracking(&enemy); break;
        
    }
}