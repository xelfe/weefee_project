#ifndef QUADRUPED_KINEMATICS_H
#define QUADRUPED_KINEMATICS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"

// Structure representing a 3D position
typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

// Structure for orientation in Euler angles
typedef struct {
    float roll;
    float pitch;
    float yaw;
} orientation_t;

// Structure representing a robot leg
typedef struct {
    // Servo IDs (coxa, femur, tibia)
    int servo_ids[3];
    
    // Leg dimensions (segment lengths)
    float coxa_length;
    float femur_length;
    float tibia_length;
    
    // Position of the leg mounting point relative to robot center
    vec3_t mounting_position;
    
    // Current foot position
    vec3_t foot_position;
    
    // Current joint angles
    float angles[3];
} leg_t;

// Constants for leg identification
#define LEG_FRONT_RIGHT   0
#define LEG_FRONT_LEFT    1
#define LEG_REAR_RIGHT    2
#define LEG_REAR_LEFT     3
#define LEG_COUNT         4

// Constants for joint identification
#define JOINT_COXA    0
#define JOINT_FEMUR   1
#define JOINT_TIBIA   2
#define JOINT_COUNT   3

// Kinematic function prototypes
esp_err_t inverse_kinematics(leg_t *leg, const vec3_t *target_pos, float angles_out[3]);
void forward_kinematics(const leg_t *leg, const float angles[3], vec3_t *position_out);
void init_kinematics(void);
void set_robot_dimensions(float body_length, float body_width, float coxa_length, float femur_length, float tibia_length);

#endif // QUADRUPED_KINEMATICS_H