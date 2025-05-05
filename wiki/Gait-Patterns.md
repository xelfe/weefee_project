# Gait Patterns

This page documents the different walking patterns (gaits) implemented for the Weefee quadruped robot.

## Introduction to Quadruped Gaits

A gait is a specific pattern of limb movements used during locomotion. For quadruped robots, different gaits provide various advantages in terms of:
- Stability
- Speed
- Energy efficiency
- Terrain adaptability

## Leg Naming Convention

The Weefee robot uses the following naming convention for its legs:

```
     Front
  FR +---+ FL
     |   |
     |   |
  RR +---+ RL
      Rear
```

Where:
- **FR**: Front Right
- **FL**: Front Left
- **RR**: Rear Right
- **RL**: Rear Left

## Gait Phases

Each leg goes through two primary phases during locomotion:

1. **Stance Phase**: The foot is on the ground and supporting the robot's weight
2. **Swing Phase**: The foot is lifted off the ground and moving forward

The coordination of these phases across all four legs defines the gait pattern.

## Implemented Gaits

### 1. Walking Gait

The walking gait is a slow, stable gait where only one leg is in the swing phase at any time.

#### Sequence:
1. Lift and move Front Right (FR) leg forward
2. Lift and move Rear Right (RR) leg forward
3. Lift and move Front Left (FL) leg forward
4. Lift and move Rear Left (RL) leg forward
5. Repeat

#### Characteristics:
- Very stable (3 feet on ground at all times)
- Slower movement
- Lower energy requirements
- Good for uneven terrain

#### Implementation:
```cpp
void calculate_walking_gait() {
    // Walking gait sequence: FR, RR, FL, RL
    // Each leg moves one after another
    
    // Define phase offsets for each leg (0.0 to 1.0)
    float phase_offsets[4] = {0.0f, 0.25f, 0.5f, 0.75f};
    
    for (int i = 0; i < 4; i++) {
        // Calculate phase for this leg
        float phase = walk_cycle_progress_ - phase_offsets[i];
        if (phase < 0.0f) phase += 1.0f;
        
        // Default position (standing)
        Vec3 foot_pos = legs_[i].mounting_position;
        foot_pos.z = -standing_height_;
        
        if (phase < 0.5f) {
            // Swing phase (foot in air)
            float swing_progress = phase / 0.5f; // 0.0 to 1.0
            
            // X movement (front to back)
            foot_pos.x += step_length_ * (0.5f - swing_progress);
            
            // Z movement (up and down)
            float height_factor = sin(swing_progress * M_PI);
            foot_pos.z += step_height_ * height_factor;
        } else {
            // Stance phase (foot on ground)
            float stance_progress = (phase - 0.5f) / 0.5f; // 0.0 to 1.0
            
            // X movement (back to front)
            foot_pos.x += step_length_ * (stance_progress - 0.5f);
        }
        
        // Store calculated position
        target_foot_positions_[i] = foot_pos;
    }
}
```

### 2. Trotting Gait

The trotting gait moves diagonal leg pairs together, providing a balance of speed and stability.

#### Sequence:
1. Lift and move diagonal pair 1 (FR + RL) forward
2. Lift and move diagonal pair 2 (FL + RR) forward
3. Repeat

#### Characteristics:
- Reasonably stable (2 feet on ground at all times)
- Faster than walking
- Moderate energy requirements
- Good for even terrain and normal operation

#### Implementation:
```cpp
void calculate_trotting_gait() {
    // Trotting gait: diagonal pairs move together
    // FR+RL, then FL+RR
    
    // Define diagonal pairs
    int diagonal_pairs[2][2] = {{0, 3}, {1, 2}}; // FR+RL, FL+RR
    
    for (int pair = 0; pair < 2; pair++) {
        // Calculate phase for this pair
        float phase = walk_cycle_progress_ - 0.5f * pair;
        if (phase < 0.0f) phase += 1.0f;
        
        for (int leg = 0; leg < 2; leg++) {
            int leg_idx = diagonal_pairs[pair][leg];
            
            // Default position (standing)
            Vec3 foot_pos = legs_[leg_idx].mounting_position;
            foot_pos.z = -standing_height_;
            
            if (phase < 0.5f) {
                // Swing phase (foot in air)
                float swing_progress = phase / 0.5f; // 0.0 to 1.0
                
                // X movement (front to back)
                foot_pos.x += step_length_ * (0.5f - swing_progress);
                
                // Z movement (up and down)
                float height_factor = sin(swing_progress * M_PI);
                foot_pos.z += step_height_ * height_factor;
            } else {
                // Stance phase (foot on ground)
                float stance_progress = (phase - 0.5f) / 0.5f; // 0.0 to 1.0
                
                // X movement (back to front)
                foot_pos.x += step_length_ * (stance_progress - 0.5f);
            }
            
            // Store calculated position
            target_foot_positions_[leg_idx] = foot_pos;
        }
    }
}
```

### 3. Standing Position

While not technically a gait, the standing position is an important default stance:

#### Characteristics:
- All legs provide support
- Body maintained at level height
- Even weight distribution
- Used as the starting and ending position for movements

#### Implementation:
```cpp
void robot_stand(float height) {
    // If height is 0 or invalid, use default height
    if (height <= 0.0f) {
        height = DEFAULT_STANDING_HEIGHT;
    }
    
    // Set all foot positions directly under their mounting points
    for (int i = 0; i < LEG_COUNT; i++) {
        target_foot_positions[i].x = leg_mounting_points[i].x;
        target_foot_positions[i].y = leg_mounting_points[i].y;
        target_foot_positions[i].z = -height;
        
        // Calculate joint angles using inverse kinematics
        inverse_kinematics(i, &target_foot_positions[i], target_joint_angles[i]);
    }
    
    // Apply calculated angles to servos
    apply_servo_positions(target_joint_angles);
}
```

## Gait Parameters

The gait patterns can be adjusted using several parameters:

### Step Height
The maximum height the foot reaches during the swing phase.
- Higher values provide better obstacle clearance
- Lower values are more energy-efficient

### Step Length
The distance between the forward and backward positions of the foot.
- Longer steps increase speed but may reduce stability
- Shorter steps provide more stability but reduce speed

### Gait Cycle Speed
How fast the robot progresses through the gait cycle.
- Higher values increase overall movement speed
- Can be adjusted on-the-fly with the "walk [speed]" command

## Gait Control via ROS2

Gaits can be controlled through ROS2 using string commands:

```bash
# Start walking gait with default speed
ros2 topic pub /robot_command std_msgs/msg/String "data: 'walk'"

# Start walking with specified speed
ros2 topic pub /robot_command std_msgs/msg/String "data: 'walk 1.5'"

# Start trotting gait
ros2 topic pub /robot_command std_msgs/msg/String "data: 'trot'"

# Stop movement
ros2 topic pub /robot_command std_msgs/msg/String "data: 'stop'"
```

## Advanced Gaits

Future versions of the Weefee project may implement more advanced gaits:

### Pace Gait
Moving the legs on the same side together.

### Bound Gait
Moving front legs together, followed by rear legs.

### Gallop Gait
A fast, asymmetrical gait with a brief period where all feet are off the ground.

### Dynamic Gait Transition
Smooth transitions between different gait patterns based on speed requirements and terrain.

## References

- [Principles of Animal Locomotion](https://press.princeton.edu/books/paperback/9780691126340/principles-of-animal-locomotion) by R. McNeill Alexander
- [Quadrupedal Gaits and Body Support Patterns](https://www.researchgate.net/publication/232784317_Quadrupedal_Gaits_and_Body_Support_Patterns)
- [MIT Cheetah Robot](https://biomimetics.mit.edu/research/mit-cheetah-robot)