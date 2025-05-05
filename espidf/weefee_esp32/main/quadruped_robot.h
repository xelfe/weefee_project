#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "esp_err.h"
#include "quadruped_kinematics.h"
#include "servo_controller.h"

// Structure representing the robot state
typedef struct {
    // Robot legs
    leg_t legs[LEG_COUNT];
    
    // Body position and orientation
    vec3_t body_position;
    orientation_t body_orientation;
    
    // Gait configuration
    float gait_height;
    float step_length;
    float step_height;
    
    // Robot state
    bool is_standing;
    bool is_walking;
    float walk_cycle_progress;
} quadruped_t;

// Gait type definitions
typedef enum {
    GAIT_STAND = 0,
    GAIT_WALK = 1,
    GAIT_TROT = 2,
    GAIT_PACE = 3
} gait_type_t;

// Function prototypes
void robot_init(void);
esp_err_t robot_set_body_position(const vec3_t *position);
esp_err_t robot_set_body_orientation(const orientation_t *orientation);
esp_err_t robot_set_leg_position(int leg_index, const vec3_t *position);
esp_err_t robot_stand(float height);
esp_err_t robot_sit(void);
esp_err_t robot_start_gait(gait_type_t gait, float speed);
esp_err_t robot_stop_gait(void);
void robot_update(void);
void robot_map_angles_to_servos(void);

#endif // QUADRUPED_ROBOT_H