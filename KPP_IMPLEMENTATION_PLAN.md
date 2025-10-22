# Kinematic Pose Position (KPP) System Implementation Plan

## Overview
This document outlines the implementation plan for adding a Kinematic Pose Position (KPP) system to the hexapod robot. The KPP system will provide state estimation, motion smoothing, and acceleration/jerk limiting to improve movement quality and reduce servo stress.

## Current System Analysis

### Existing Architecture
- **Main Loop**: 10ms control cycle in `gait_framework_main()`
- **Control Flow**: `user_command` → `gait_scheduler` → `swing_trajectory` → `whole_body_control` → `robot_execute`
- **Joint Control**: Direct servo angle commands via `robot_set_joint_angle_rad()`
- **Coordinate Frames**: 
  - Body frame: X forward, Y left, Z up
  - Leg-local frame: X outward, Y forward, Z up

### Missing Components
1. No kinematic state estimation
2. No motion smoothing between poses
3. No acceleration/jerk limiting
4. No feedback from actual servo positions

## Implementation Strategy

### Phase 1: Core KPP Infrastructure
**Goal**: Establish foundation for state estimation and motion limiting

#### 1.1 Data Structures

```c
// New file: main/kpp_system.h
typedef enum {
    MOTION_MODE_NORMAL,      // S-curve jerk-limited, very smooth
    // TODO: Add MOTION_MODE_FAST and MOTION_MODE_EMERGENCY later
} motion_mode_t;

typedef struct {
    // Joint state (6 legs × 3 joints each)
    float joint_angles[6][3];      // Current angles [leg][joint] (rad)
    float joint_velocities[6][3];  // Angular velocities (rad/s)
    float joint_accelerations[6][3]; // Angular accelerations (rad/s²)
    
    // Leg endpoint positions in body frame
    float leg_positions[6][3];     // [leg][x,y,z] (m)
    float leg_velocities[6][3];    // [leg][vx,vy,vz] (m/s)
    
    // Body state (estimated from leg positions)
    float body_position[3];        // x, y, z in world frame
    float body_orientation[3];     // roll, pitch, yaw (rad)
    float body_velocity[3];        // linear velocity (m/s)
    float body_angular_vel[3];     // angular velocity (rad/s)
    
    // Timing
    float last_update_time;        // For dt calculation
    bool initialized;              // First update flag
} kinematic_state_t;

typedef struct {
    // Motion limits per joint type [coxa, femur, tibia]
    float max_velocity[3];         // rad/s
    float max_acceleration[3];     // rad/s²
    float max_jerk[3];            // rad/s³
    
    motion_mode_t current_mode;
    // TODO: Add mode_transition_time for future multi-mode support
} motion_limits_t;
```

#### 1.2 Core Functions

```c
// New file: main/kpp_system.c

// Initialize KPP system
esp_err_t kpp_init(kinematic_state_t* state, motion_limits_t* limits);

// Update kinematic state from current servo positions
void kpp_update_state(kinematic_state_t* state, const whole_body_cmd_t* current_cmd, float dt);

// Apply motion limits to desired joint commands
void kpp_apply_limits(const kinematic_state_t* state, const motion_limits_t* limits, 
                          const whole_body_cmd_t* desired_cmd, whole_body_cmd_t* limited_cmd, float dt);

// Forward kinematics: joint angles → leg positions
esp_err_t kpp_forward_kinematics(const float joint_angles[6][3], float leg_positions[6][3]);

// Set motion mode (normal/fast/emergency)
esp_err_t kpp_set_motion_mode(motion_limits_t* limits, motion_mode_t mode);
```

#### 1.3 Integration Points

**Modified Files**:
- `main/main.c`: Add KPP system initialization and update calls
- `main/robot_control.c`: Integrate motion limiting before servo commands
- `main/CMakeLists.txt`: Add new KPP source files

### Phase 2: State Estimation Implementation
**Goal**: Track actual robot state based on commanded positions

#### 2.1 State Update Algorithm
1. **Joint State**: Use commanded angles as best estimate (Phase 1)
2. **Velocity Estimation**: Numerical differentiation with filtering
3. **Acceleration Estimation**: Second derivative with noise filtering
4. **Forward Kinematics**: Compute leg positions from joint angles
5. **Body Pose**: Estimate from leg positions and contact states

#### 2.2 Filtering and Smoothing
- Low-pass filters for velocity/acceleration estimation
- Configurable filter cutoff frequencies
- Handle first-update initialization properly

### Phase 3: Motion Limiting
**Goal**: Smooth motion with S-curve jerk limiting

#### 3.1 S-Curve Motion Planning Algorithm
```c
// S-curve trajectory generation for smooth, jerk-limited motion
// Implementation will use exponential smoothing or simple S-curve approximation
// TODO: Evaluate exponential vs trapezoidal vs full S-curve after initial testing

// For each joint:
float desired_velocity = (target_angle - current_angle) / dt;
float desired_accel = (desired_velocity - current_velocity) / dt;
float desired_jerk = (desired_accel - current_accel) / dt;

// Apply limits in order: jerk → acceleration → velocity
if (abs(desired_jerk) > max_jerk) {
    desired_accel = current_accel + sign(desired_jerk) * max_jerk * dt;
}
if (abs(desired_accel) > max_accel) {
    desired_velocity = current_velocity + sign(desired_accel) * max_accel * dt;
}
if (abs(desired_velocity) > max_velocity) {
    desired_velocity = sign(desired_velocity) * max_velocity;
}

// Integrate to get limited target angle
limited_target = current_angle + desired_velocity * dt;
```

#### 3.2 Logging and Debugging
- KPP state logged with `ESP_LOGD()` for development debugging
- Configurable log level for performance tuning

### Phase 4: Enhanced Features (Future)
**Goal**: Advanced capabilities built on KPP foundation

#### 4.1 Multi-Mode Motion Profiles
**NOTE**: Starting with Normal mode only. Fast and Emergency modes to be added later.

- **A) Normal Walk**: S-curve jerk-limited, focus on acceleration/jerk control
  - vmax = 6.0 rad/s (near hardware limit), a = 50 rad/s², j = 2,000 rad/s³
  - Purpose: smooth motion via acceleration/jerk limiting, not velocity restriction
- **B) Fast Reaction**: Responsive but safe (TODO - future)
  - vmax = 4.50 rad/s, a = 250 rad/s², j = 15,000 rad/s³
- **C) Rapid Reaction**: Hardware limit emergency mode (TODO - future)
  - vmax = 8.0 rad/s, a = 350 rad/s², j = 35,000 rad/s³

#### 4.2 Sensor Integration Preparation
- IMU integration interface
- Force sensor integration interface
- Sensor fusion algorithms

## Implementation Details

### Default Motion Limits (Initial Values)
```c
// Normal mode limits (S-curve jerk-limited profile)
// Focus on acceleration and jerk limits, allow higher velocity
motion_limits_t default_limits = {
    .max_velocity = {6.0f, 6.0f, 6.0f},         // rad/s [coxa, femur, tibia] - near hardware limit
    .max_acceleration = {50.0f, 50.0f, 50.0f},  // rad/s²
    .max_jerk = {2000.0f, 2000.0f, 2000.0f},    // rad/s³
    .current_mode = MOTION_MODE_NORMAL,
};

// TODO: Configuration will be compile-time constants initially.
// Later add NVS storage for runtime tuning.
```

### Integration with Existing Control Loop
```c
// Modified main.c gait_framework_main()
void gait_framework_main(void *arg) {
    // ... existing initialization ...
    
    // Add KPP initialization
    kinematic_state_t kpp_state;
    motion_limits_t motion_limits;
    kpp_init(&kpp_state, &motion_limits);
    
    while (1) {
        // ... existing command polling ...
        
        // Generate desired commands (existing path)
        gait_scheduler_update(&scheduler, dt, &ucmd);
        swing_trajectory_generate(&trajectory, &scheduler, &ucmd);
        whole_body_control_compute(&trajectory, &cmds);
        
        // NEW: Apply KPP motion limiting
        whole_body_cmd_t limited_cmds;
        kpp_apply_limits(&kpp_state, &motion_limits, &cmds, &limited_cmds, dt);
        
        // Execute limited commands
        robot_execute(&limited_cmds);
        
        // NEW: Update KPP state estimation
        kpp_update_state(&kpp_state, &limited_cmds, dt);
        
        // ... existing timing ...
    }
}
```

## Decision Points Requiring Confirmation

### ✅ Resolved Decisions:
1. **Motion Modes**: Starting with Normal mode only (S-curve, jerk-limited)
2. **Configuration**: Compile-time constants initially, TODO for NVS storage later
3. **Logging**: KPP state logged with ESP_LOGD() for debugging
4. **Filter Type**: Simple strategy (exponential or S-curve approximation), to be refined later
5. **Motion Profile**: Normal walk profile with specified limits (1.5 rad/s, 50 rad/s², 2000 rad/s³)

### 🔍 Remaining Decisions:

### 🔍 Remaining Decisions:

1. **Filter Parameters**
- **Question**: What cutoff frequency for velocity/acceleration filters?
- **Proposal**: 10 Hz cutoff (conservative, adjustable via config)

2. **Coordinate Frame Consistency**
- **Question**: Confirm all kinematic calculations use consistent frames?
- **Current**: Body frame (X forward, Y left, Z up) for high-level, leg-local for IK

3. **Memory Allocation**
- **Question**: Static allocation vs dynamic for KPP structures?
- **Proposal**: Static allocation for real-time safety

## File Structure
```
main/
├── kpp_system.h          # KPP data structures and API
├── kpp_system.c          # Core KPP implementation
├── kpp_forward_kin.c     # Forward kinematics calculations
├── kpp_motion_limits.c   # Motion limiting algorithms
├── kpp_config.h          # Configurable parameters
└── (existing files...)   # Modified to integrate KPP
```

## Testing Strategy

### Phase 1 Testing
1. **Unit Tests**: Individual KPP functions with known inputs
2. **Integration Test**: KPP in existing control loop with very high limits (no limiting)
3. **Limit Verification**: Gradually reduce limits, verify smooth motion

### Phase 2 Testing
1. **State Accuracy**: Compare estimated vs commanded positions
2. **Velocity Estimation**: Validate velocity calculations
3. **Motion Smoothness**: Visual/audio assessment of servo stress

### Phase 3 Testing
1. **Limit Compliance**: Verify acceleration/jerk stay within bounds
2. **Response Time**: Measure command-to-motion latency
3. **Mode Switching**: Test transitions between motion modes

## Timeline Estimate
- **Phase 1 (Infrastructure)**: 2-3 days
- **Phase 2 (State Estimation)**: 2-3 days  
- **Phase 3 (Motion Limiting)**: 3-4 days
- **Testing & Tuning**: 2-3 days
- **Total**: 9-13 days

## Questions for Confirmation

1. **Are the remaining filter and coordinate frame decisions acceptable?**
2. **Should we start implementation with Phase 1 infrastructure?**
3. **Any other considerations before we begin coding?**

**Implementation Ready**: All major decisions resolved. Your motion profiles are excellent - the Normal walk profile will provide very smooth, servo-friendly motion perfect for continuous gait operation.