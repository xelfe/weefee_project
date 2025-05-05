# Quadruped Kinematics

This page explains the kinematics implementation used in the Weefee quadruped robot.

## Introduction to Robot Kinematics

Kinematics is the mathematical description of motion without considering the forces that cause it. For a quadruped robot, kinematics is essential for:

1. **Forward Kinematics**: Calculating where the foot will be based on joint angles
2. **Inverse Kinematics**: Calculating joint angles needed to place the foot at a specific position

## Leg Configuration

The Weefee robot uses a standard 3-joint leg configuration per leg:

1. **Coxa**: The hip joint that rotates horizontally (connecting to the body)
2. **Femur**: The middle joint between the coxa and tibia
3. **Tibia**: The final segment connecting to the foot

![Leg Diagram](https://i.imgur.com/example-placeholder.jpg)

Each leg has three degrees of freedom (DoF), giving the robot a total of 12 DoF for its four legs.

## Coordinate System

The robot uses a right-handed coordinate system:
- **X-axis**: Forward/backward direction
- **Y-axis**: Left/right direction
- **Z-axis**: Up/down direction

The origin is at the center of the robot's body.

## Forward Kinematics

Forward kinematics calculates the foot position given the joint angles.

### Mathematical Approach

For a leg with joint angles θ₁ (coxa), θ₂ (femur), and θ₃ (tibia):

1. Calculate the position after coxa rotation:
   ```
   x_coxa = L_coxa * cos(θ₁)
   y_coxa = L_coxa * sin(θ₁)
   z_coxa = 0
   ```

2. Calculate the femur contribution:
   ```
   x_femur = L_femur * cos(θ₂) * cos(θ₁)
   y_femur = L_femur * cos(θ₂) * sin(θ₁)
   z_femur = -L_femur * sin(θ₂)
   ```

3. Calculate the tibia contribution:
   ```
   x_tibia = L_tibia * cos(θ₃ + θ₂) * cos(θ₁)
   y_tibia = L_tibia * cos(θ₃ + θ₂) * sin(θ₁)
   z_tibia = -L_tibia * sin(θ₃ + θ₂)
   ```

4. Sum all contributions to get the final foot position:
   ```
   x = mounting_x + x_coxa + x_femur + x_tibia
   y = mounting_y + y_coxa + y_femur + y_tibia
   z = mounting_z + z_coxa + z_femur + z_tibia
   ```

Where:
- `L_coxa`, `L_femur`, `L_tibia` are the lengths of each segment
- `mounting_x/y/z` is the position where the leg connects to the body

### Implementation

The forward kinematics is implemented in both the ESP32 firmware (`quadruped_kinematics.c`) and the ROS2 code (`quadruped_inverse_kinematics.h`).

## Inverse Kinematics

Inverse kinematics calculates the joint angles required to position the foot at a specific location.

### Mathematical Approach

For a target foot position (x, y, z) relative to the leg's mounting point:

1. Calculate the horizontal distance from leg base to foot:
   ```
   L = sqrt(x² + y²)
   ```

2. Calculate the coxa angle:
   ```
   θ₁ = atan2(y, x)
   ```

3. Remove the coxa length from the horizontal distance:
   ```
   L2 = L - L_coxa
   ```

4. Calculate the straight-line distance from femur joint to foot:
   ```
   L_femur_tibia = sqrt(L2² + z²)
   ```

5. Check if the position is reachable (L_femur_tibia ≤ L_femur + L_tibia)

6. Using the law of cosines, calculate:
   ```
   gamma = atan2(z, L2)
   alpha = acos((L_femur² + L_femur_tibia² - L_tibia²) / (2 * L_femur * L_femur_tibia))
   beta = acos((L_femur² + L_tibia² - L_femur_tibia²) / (2 * L_femur * L_tibia))
   ```

7. Calculate the final joint angles:
   ```
   θ₂ (femur) = 90° - (gamma + alpha)
   θ₃ (tibia) = 180° - beta
   ```

### Implementation

The inverse kinematics is implemented in both the ESP32 firmware and ROS2 control code.

Key functions:
- `inverse_kinematics()` in `quadruped_kinematics.c` (ESP32)
- `inverse_kinematics()` in `quadruped_inverse_kinematics.h` (ROS2)

## Body Movements

The kinematics system also handles whole-body movements:

1. **Translation**: Moving the body position while adjusting all legs to maintain the same foot positions relative to the ground
2. **Rotation**: Rotating the body while adjusting leg positions to maintain foot positions

This is achieved by:
1. Converting the desired global foot positions to body-relative positions
2. Applying the inverse transformation of the body movement
3. Calculating inverse kinematics for each leg

## Gait Generation

Gaits are created by generating foot position trajectories over time:

1. **Stance Phase**: When the foot is on the ground, moving backward
2. **Swing Phase**: When the foot is lifted and moving forward

Different gait patterns (walk, trot) vary in how these phases are coordinated across legs.

The implementation supports:
- **Walking Gait**: One leg at a time (sequence: FR → RR → FL → RL)
- **Trotting Gait**: Diagonal legs move together (FR+RL, then FL+RR)

## Code Example: Inverse Kinematics

Here's a simplified version of the inverse kinematics calculation:

```cpp
bool inverse_kinematics(int leg_index, const Vec3 &target_pos, float angles_out[3]) {
    Leg &leg = legs_[leg_index];
    
    // Calculate leg-relative position
    float leg_x = target_pos.x - leg.mounting_position.x;
    float leg_y = target_pos.y - leg.mounting_position.y;
    float leg_z = target_pos.z - leg.mounting_position.z;
    
    // Horizontal distance from leg base to foot
    float L = sqrtf(leg_x * leg_x + leg_y * leg_y);
    
    // Distance for coxa (first segment)
    float L_coxa = leg.coxa_length;
    
    // Calculate coxa angle (horizontal rotation)
    float coxa_angle = atan2f(leg_y, leg_x) * 180.0f / M_PI;
    
    // Adjust positions for femur and tibia calculations
    float L2 = L - L_coxa;
    
    // Distance for femur/tibia joints
    float L_femur_tibia = sqrtf(L2 * L2 + leg_z * leg_z);
    
    // Check if position is reachable
    if (L_femur_tibia > (leg.femur_length + leg.tibia_length)) {
        return false; // Target position out of reach
    }
    
    // Calculate angles using law of cosines
    float a = leg.femur_length;
    float b = leg.tibia_length;
    float c = L_femur_tibia;
    
    float gamma = atan2f(leg_z, L2) * 180.0f / M_PI;
    float alpha = acosf((a*a + c*c - b*b) / (2.0f * a * c)) * 180.0f / M_PI;
    float beta = acosf((a*a + b*b - c*c) / (2.0f * a * b)) * 180.0f / M_PI;
    
    // Final angle calculations
    float femur_angle = 90.0f - (gamma + alpha);
    float tibia_angle = 180.0f - beta;
    
    // Assign results
    angles_out[JOINT_COXA] = coxa_angle;
    angles_out[JOINT_FEMUR] = femur_angle;
    angles_out[JOINT_TIBIA] = tibia_angle;
    
    return true;
}
```

## Resources

For those wanting to learn more about quadruped kinematics:

- [Introduction to Robotics: Mechanics and Control](https://www.pearson.com/us/higher-education/program/Craig-Introduction-to-Robotics-Mechanics-and-Control-4th-Edition/PGM91709.html) by John J. Craig
- [Robot Academy: Inverse Kinematics](https://robotacademy.net.au/masterclass/inverse-kinematics/)
- [Quadruped Robot Gait Generation](https://www.researchgate.net/publication/264159259_Quadruped_Robot_Gait_Generation)