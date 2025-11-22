# Hexapod Configuration Persistence Design

## Overview

This document outlines the design decisions for the hexapod robot's configuration persistence layer, including storage method selection, RPC architecture, and migration strategy.

## Storage Method Selection

### Requirements Analysis

**Must Have:**
- Fast read access for real-time parameter updates
- Easy development and debugging
- Access via RPC for read/write operations  
- Space for 200+ configuration values
- Live parameter updates with immediate robot response

**Nice to Have:**
- Dead easy development workflow
- Filesystem-like access with named parameters
- Parameter segregation/organization (namespaces)

### Storage Options Evaluated

#### 1. NVS (Non-Volatile Storage) ✅ **CHOSEN**

**Advantages:**
- **Optimal for key-value configuration data** - designed exactly for this use case
- **Fast random access** - O(1) key lookups with minimal overhead
- **Built-in namespace support** - perfect for parameter organization
- **Automatic wear leveling** and power-loss protection
- **Simple API** - `nvs_set_*()` and `nvs_get_*()` functions
- **Type safety** - built-in support for int8, int16, int32, float, string, blob
- **Perfect for RPC** - keys are strings, ideal for API endpoints
- **Atomic operations** - transactional safety
- **Minimal overhead** - efficient storage with small per-parameter cost

**Storage Requirements:**
- 200+ config values ≈ 10-20KB total
- Recommended 64KB NVS partition (allows growth + wear leveling)

#### 2. FAT Filesystem ❌ **REJECTED**

**Disadvantages:**
- **Slow access** - file open/read/parse overhead for each parameter
- **Complex development** - file management, parsing, serialization required
- **Write amplification** - changing one parameter requires rewriting entire file
- **No built-in wear leveling** for small frequent writes
- **Corruption prone** on power loss during writes
- **Overkill** for configuration data
- **Poor RPC performance** - would require file parsing for each parameter access

#### 3. SPIFFS/LittleFS ❌ **NOT SUITABLE**

**Why Not Considered:**
- **SPIFFS**: Deprecated in ESP-IDF, wear leveling issues, slower than NVS for key-value access
- **LittleFS**: Better than SPIFFS but still filesystem overhead, designed for file storage not configuration
- **Both suffer from**: File-based access overhead, complex parameter management, no built-in key-value semantics

### NVS Namespace Organization

Following Betaflight's Parameter Group (PG) concept, organized by logical functionality:

```c
// Primary configuration namespaces
#define CONFIG_NS_JOINT_CALIB    "joint_cal"    // Joint calibration per leg/joint
#define CONFIG_NS_LEG_GEOMETRY   "leg_geom"     // Leg link lengths and mounting poses
#define CONFIG_NS_MOTION_LIMITS  "motion_lim"   // KPP velocity/acceleration/jerk limits  
#define CONFIG_NS_SERVO_MAPPING  "servo_map"    // GPIO pins and driver selections
#define CONFIG_NS_CONTROLLER     "controller"   // Controller driver configurations
#define CONFIG_NS_GAIT           "gait"         // Gait scheduler and trajectory parameters
#define CONFIG_NS_SYSTEM         "system"       // System-wide settings and safety
#define CONFIG_NS_DEBUG          "debug"        // Debug and logging parameters
#define CONFIG_NS_WIFI           "wifi"         // WiFi AP and network configuration
```

## Detailed Namespace Design

### 1. CONFIG_NS_JOINT_CALIB - Joint Calibration Data
**Derived from:** `robot_config_t.joint_calib[NUM_LEGS][3]`

**Parameters per leg/joint:**
```c
// Joint calibration parameters (18 parameters per leg × 6 legs = 108 total)
"leg{0-5}_coxa_offset"      // joint_calib_t.zero_offset_rad
"leg{0-5}_coxa_invert"      // joint_calib_t.invert_sign  
"leg{0-5}_coxa_min"         // joint_calib_t.min_rad
"leg{0-5}_coxa_max"         // joint_calib_t.max_rad
"leg{0-5}_coxa_pwm_min"     // joint_calib_t.pwm_min_us
"leg{0-5}_coxa_pwm_max"     // joint_calib_t.pwm_max_us
"leg{0-5}_coxa_pwm_neutral" // joint_calib_t.neutral_us

// Same pattern for femur and tibia joints
"leg{0-5}_femur_offset", "leg{0-5}_femur_invert", etc.
"leg{0-5}_tibia_offset", "leg{0-5}_tibia_invert", etc.
```

### 2. CONFIG_NS_LEG_GEOMETRY - Leg Physical Properties  
**Derived from:** `leg_geometry_t` and mounting poses

**Leg geometry parameters:**
```c
// Mechanical dimensions (18 parameters per leg × 6 legs = 108 total)
"leg{0-5}_len_coxa"         // leg_geometry_t.len_coxa (meters)
"leg{0-5}_len_femur"        // leg_geometry_t.len_femur (meters)  
"leg{0-5}_len_tibia"        // leg_geometry_t.len_tibia (meters)

// Mounting poses in body frame (24 parameters per leg × 6 legs = 144 total)
"leg{0-5}_mount_x"          // Base pose X offset (meters)
"leg{0-5}_mount_y"          // Base pose Y offset (meters)
"leg{0-5}_mount_z"          // Base pose Z offset (meters)  
"leg{0-5}_mount_yaw"        // Base pose yaw rotation (radians)

// Stance positions in leg-local frame
"leg{0-5}_stance_out"       // Outward stance distance (meters)
"leg{0-5}_stance_fwd"       // Forward stance distance (meters)
```

### 3. CONFIG_NS_MOTION_LIMITS - KPP Motion Parameters
**Derived from:** `kpp_config.h` motion limits and filters

**Motion limits per joint type:**
```c
// Joint motion limits (9 parameters × 3 joint types = 27 total)  
"max_velocity_coxa"         // KPP_MAX_VELOCITY_COXA (rad/s)
"max_velocity_femur"        // KPP_MAX_VELOCITY_FEMUR (rad/s)
"max_velocity_tibia"        // KPP_MAX_VELOCITY_TIBIA (rad/s)
"max_acceleration_coxa"     // KPP_MAX_ACCELERATION_COXA (rad/s²)
"max_acceleration_femur"    // KPP_MAX_ACCELERATION_FEMUR (rad/s²) 
"max_acceleration_tibia"    // KPP_MAX_ACCELERATION_TIBIA (rad/s²)
"max_jerk_coxa"            // KPP_MAX_JERK_COXA (rad/s³)
"max_jerk_femur"           // KPP_MAX_JERK_FEMUR (rad/s³)
"max_jerk_tibia"           // KPP_MAX_JERK_TIBIA (rad/s³)

// State estimation filters (~15 parameters)
"velocity_filter_alpha"     // KPP_VELOCITY_FILTER_ALPHA
"accel_filter_alpha"        // KPP_ACCEL_FILTER_ALPHA
"leg_velocity_filter_alpha" // KPP_LEG_VELOCITY_FILTER_ALPHA
"body_velocity_filter_alpha"// KPP_BODY_VELOCITY_FILTER_ALPHA
"body_pitch_filter_alpha"   // KPP_BODY_PITCH_FILTER_ALPHA
"body_roll_filter_alpha"    // KPP_BODY_ROLL_FILTER_ALPHA

// Geometric and validation parameters
"front_to_back_distance"    // KPP_FRONT_TO_BACK_DISTANCE (meters)
"left_to_right_distance"    // KPP_LEFT_TO_RIGHT_DISTANCE (meters)
"max_leg_velocity"          // KPP_MAX_LEG_VELOCITY (m/s)
"max_body_velocity"         // KPP_MAX_BODY_VELOCITY (m/s) 
"max_angular_velocity"      // KPP_MAX_ANGULAR_VELOCITY (rad/s)
"min_dt"                    // KPP_MIN_DT (seconds)
"max_dt"                    // KPP_MAX_DT (seconds)

// Body frame offsets
"body_offset_x"             // KPP_BODY_OFFSET_X (meters)
"body_offset_y"             // KPP_BODY_OFFSET_Y (meters)
"body_offset_z"             // KPP_BODY_OFFSET_Z (meters)
```

### 4. CONFIG_NS_SERVO_MAPPING - Hardware Configuration
**Derived from:** `robot_config_t` servo and MCPWM mappings

**Servo hardware mapping:**
```c
// GPIO pin assignments (18 parameters per leg × 6 legs = 108 total)
"leg{0-5}_coxa_gpio"        // servo_gpio[leg][LEG_SERVO_COXA] 
"leg{0-5}_femur_gpio"       // servo_gpio[leg][LEG_SERVO_FEMUR]
"leg{0-5}_tibia_gpio"       // servo_gpio[leg][LEG_SERVO_TIBIA]

// Driver selection (18 parameters per leg × 6 legs = 108 total)
"leg{0-5}_coxa_driver"      // servo_driver_sel[leg][LEG_SERVO_COXA] (0=MCPWM, 1=LEDC)
"leg{0-5}_femur_driver"     // servo_driver_sel[leg][LEG_SERVO_FEMUR]
"leg{0-5}_tibia_driver"     // servo_driver_sel[leg][LEG_SERVO_TIBIA]

// MCPWM group assignment (6 parameters)
"leg{0-5}_mcpwm_group"      // mcpwm_group_id[leg]
```

### 5. CONFIG_NS_CONTROLLER - Input Device Configuration
**Derived from:** `controller_config_t` and driver-specific configs

**Controller driver configuration:**
```c
// Core controller settings (~8 parameters)
"driver_type"               // controller_driver_type_e
"task_stack"               // task_stack (bytes)
"task_priority"            // task_prio

// FlySky iBUS driver settings (~6 parameters)
"flysky_uart_port"         // controller_flysky_ibus_cfg_t.uart_port
"flysky_tx_gpio"           // controller_flysky_ibus_cfg_t.tx_gpio
"flysky_rx_gpio"           // controller_flysky_ibus_cfg_t.rx_gpio
"flysky_rts_gpio"          // controller_flysky_ibus_cfg_t.rts_gpio
"flysky_cts_gpio"          // controller_flysky_ibus_cfg_t.cts_gpio
"flysky_baud_rate"         // controller_flysky_ibus_cfg_t.baud_rate

// Channel mapping and calibration (~20 parameters)
"channel_deadzone"         // Dead zone for stick inputs
"channel_expo"             // Exponential curve factor
"stick_calibration_min"    // Minimum stick values per channel
"stick_calibration_max"    // Maximum stick values per channel
"switch_thresholds"        // 3-position switch thresholds
```

### 6. CONFIG_NS_GAIT - Locomotion Parameters  
**Derived from:** `gait_scheduler_t` and `swing_trajectory_t`

**Gait and trajectory parameters:**
```c
// Gait timing (~8 parameters)
"cycle_time"               // gait_scheduler_t.cycle_time (seconds)
"default_gait_mode"        // Default gait type (tripod/wave/ripple)

// Swing trajectory parameters (~12 parameters)
"step_length"              // swing_trajectory_t.step_length (meters)
"clearance_height"         // swing_trajectory_t.clearance_height (meters)
"y_range_m"                // swing_trajectory_t.y_range_m (meters)
"z_min_m"                  // swing_trajectory_t.z_min_m (meters)
"z_max_m"                  // swing_trajectory_t.z_max_m (meters)

// Future gait parameters (noted in TODOs)
"max_turn_angle"           // Maximum turn angle (radians)
"turn_direction"           // Turn direction preference
"step_height_scale"        // Dynamic step height scaling
"stance_width_scale"       // Dynamic stance width scaling
```

### 7. CONFIG_NS_SYSTEM - System-Wide Settings
**Derived from:** System safety and operational parameters

**System configuration:**
```c
// Safety settings (~10 parameters)
"emergency_stop_enabled"    // Emergency stop functionality
"auto_disarm_timeout"       // Auto-disarm timeout (seconds)
"safety_voltage_min"        // Minimum battery voltage (volts)
"temperature_limit_max"     // Maximum operating temperature (°C)
"motion_timeout_ms"         // Motion command timeout
"startup_delay_ms"          // Startup safety delay
"max_control_frequency"     // Maximum control loop frequency (Hz)

// System identification
"robot_id"                  // Unique robot identifier
"robot_name"                // Human-readable robot name
"config_version"            // Configuration schema version
```

### 8. CONFIG_NS_DEBUG - Development and Diagnostic Settings
**Derived from:** `robot_config_t` debug settings and `kpp_config.h` logging

**Debug and logging configuration:**
```c
// Debug control (~8 parameters)
"debug_leg_enable"          // robot_config_t.debug_leg_enable
"debug_leg_index"           // robot_config_t.debug_leg_index  
"debug_leg_delta_thresh"    // robot_config_t.debug_leg_delta_thresh
"debug_leg_min_interval_ms" // robot_config_t.debug_leg_min_interval_ms

// Logging settings (~10 parameters)
"enable_state_logging"      // KPP_ENABLE_STATE_LOGGING
"log_interval"              // KPP_LOG_INTERVAL (control cycles)
"enable_limit_logging"      // KPP_ENABLE_LIMIT_LOGGING
"enable_debug_monitoring"   // KPP_ENABLE_DEBUG_MONITORING
"debug_log_interval"        // KPP_DEBUG_LOG_INTERVAL (control cycles)
"log_level"                 // ESP log level (ERROR/WARN/INFO/DEBUG)
"telemetry_rate"            // Telemetry broadcast rate (Hz)
"enable_performance_stats"  // Performance monitoring enable
```

### 9. CONFIG_NS_WIFI - Network Configuration
**Derived from:** `wifi_ap_options_t` and future network settings

**WiFi and network parameters:**
```c
// WiFi AP settings (~10 parameters)
"ap_ssid_mode"             // wifi_ap_ssid_mode_e
"ap_fixed_prefix"          // SSID prefix for MAC/random modes
"ap_fixed_ssid"            // Fixed SSID when mode == FIXED
"ap_password"              // WPA2 password (or NULL for open)
"ap_channel"               // WiFi channel (1-13)
"ap_max_clients"           // Maximum concurrent clients

// Network services (~8 parameters)
"web_server_port"          // HTTP server port (default 80)
"websocket_port"           // WebSocket server port  
"telemetry_port"           // UDP telemetry port
"api_rate_limit"           // API request rate limit (req/sec)
"enable_ota_updates"       // Over-the-air update capability
"mdns_hostname"            // mDNS hostname (hexapod.local)
```

## Summary Statistics

**Total Estimated Parameters:** ~600-700 parameters
- Joint Calibration: ~126 parameters (18 × 6 legs + extras)
- Leg Geometry: ~108 parameters (18 × 6 legs)  
- Motion Limits: ~27 parameters
- Servo Mapping: ~114 parameters (19 × 6 legs)
- Controller: ~35 parameters
- Gait: ~20 parameters
- System: ~15 parameters
- Debug: ~18 parameters
- WiFi: ~18 parameters

**Storage Requirements:** ~35-50KB for all parameters (well within 64KB NVS partition)

## RPC Architecture (Draft)

### Dual Method Approach

**Simplified design without transaction complexity:**

```c
// Memory-only operations (immediate effect, no persistence)
esp_err_t config_set_joint_offset_memory(int leg_index, leg_servo_t joint, float offset_rad);
esp_err_t config_set_motion_limit_memory(leg_servo_t joint, float max_velocity);

// Persistent operations (immediate effect + save to NVS)  
esp_err_t config_set_joint_offset_persist(int leg_index, leg_servo_t joint, float offset_rad);
esp_err_t config_set_motion_limit_persist(leg_servo_t joint, float max_velocity);

// Batch operations (best of both worlds)
esp_err_t config_save_namespace(config_namespace_t ns);     // Save all memory changes
esp_err_t config_reload_namespace(config_namespace_t ns);   // Discard changes, reload from NVS
```

### RPC Protocol Design

**JSON-based RPC for configurator portal:**

```json
// Live tuning - immediate visual feedback, no flash wear
{
  "cmd": "set_joint_offset_memory",
  "leg": 0,
  "joint": "coxa", 
  "value": 0.523
}

// User likes the result, saves it permanently
{
  "cmd": "set_joint_offset_persist", 
  "leg": 0,
  "joint": "coxa",
  "value": 0.523
}

// Batch save multiple changes at once
{
  "cmd": "save_namespace",
  "namespace": "joint_calib"
}

// Revert all unsaved changes
{
  "cmd": "reload_namespace", 
  "namespace": "joint_calib"
}
```

### RPC-to-NVS Integration

**Generic parameter access for configurator discovery:**

```c
// RPC commands map directly to NVS operations
esp_err_t config_get_parameter_float(const char *namespace_str, const char *key, float *value);
esp_err_t config_set_parameter_float(const char *namespace_str, const char *key, float value);

// Example RPC endpoints:
// GET  /api/config/joint_calib/leg0_coxa_offset
// PUT  /api/config/joint_calib/leg0_coxa_offset  {"value": 0.523, "persist": false}
// POST /api/config/joint_calib/save
// POST /api/config/joint_calib/reload
```

**Configuration state management:**
```c
typedef struct {
    bool namespace_dirty[CONFIG_NS_COUNT];   // Which namespaces have unsaved changes
    bool namespace_loaded[CONFIG_NS_COUNT];  // Which namespaces are in memory cache
} config_simple_state_t;
```

### Configurator Workflows

**Live Tuning Workflow:**
1. User slides joint offset → `set_joint_offset_memory()` → robot moves immediately  
2. User continues adjusting → more `_memory()` calls → immediate feedback
3. User likes result → `save_namespace("joint_calib")` → persisted to NVS
4. OR user doesn't like it → `reload_namespace("joint_calib")` → back to saved state

**Cautious Workflow:**  
1. User changes value → `set_joint_offset_persist()` → immediate save to NVS
2. Each change is automatically safe

**Batch Workflow:**
1. User makes multiple memory changes  
2. Tests robot behavior with all changes
3. Either saves all (`save_namespace`) or reloads all (`reload_namespace`)

## Configuration Migration Strategy

### Sequential In-Firmware Migration

**Database-style migration approach:**

```c
#define CURRENT_CONFIG_VERSION 5

const migration_step_t migrations[] = {
    {.from = 1, .to = 2, .migrate_fn = migrate_v1_to_v2},
    {.from = 2, .to = 3, .migrate_fn = migrate_v2_to_v3},
    {.from = 3, .to = 4, .migrate_fn = migrate_v3_to_v4}, 
    {.from = 4, .to = 5, .migrate_fn = migrate_v4_to_v5},
};

esp_err_t config_manager_migrate(uint16_t detected_version) {
    for (int i = 0; i < ARRAY_SIZE(migrations); i++) {
        if (migrations[i].from == detected_version) {
            ESP_ERROR_CHECK(migrations[i].migrate_fn());
            detected_version = migrations[i].to;
            config_save_version(detected_version);  // Save intermediate state
        }
    }
    return ESP_OK;
}
```

**Migration Safety Features:**
- **Automatic backup** before migration starts
- **Rollback capability** if migration fails  
- **Intermediate state saving** to handle power loss during migration
- **Validation** after each migration step

### Backup/Restore System

**Versioned JSON export/import:**

```json
{
  "metadata": {
    "version": 5,
    "firmware": "1.2.3",
    "timestamp": 1699200000,
    "robot_id": "hexapod_001"
  },
  "joint_calib": {
    "leg0_coxa_offset": 0.523,
    "leg0_coxa_min": -1.57,
    "leg0_coxa_max": 1.57
  },
  "motion_limits": {
    "kpp_max_vel_coxa": 5.0,
    "kpp_max_accel_coxa": 600.0
  }
}
```

**External Backup Migration:**
- Python script mirrors firmware migration logic
- Enables backup migration without robot hardware
- Slight code duplication but worth it for robustness

## Advantages Over Betaflight's Approach

### Betaflight Configuration Limitations:
- **Bulk operations only** - saves entire parameter groups as binary blobs
- **No individual parameter access** - cannot change single values efficiently  
- **Flash sector management** - complex wear leveling and corruption risks
- **CLI/MSP bottleneck** - configurator must read/modify/write entire groups

### Hexapod Configuration Advantages:
- **Individual parameter persistence** - instant single-value saves via NVS
- **Live parameter updates** - immediate robot response during tuning
- **Selective persistence** - memory vs permanent storage choice
- **Much more responsive** configurator experience
- **Automatic wear leveling** handled by ESP-IDF NVS layer
- **Simple key-value model** - easier debugging and development

## Implementation Status

### Phase 1: Core Infrastructure ✅ **COMPLETED**
1. **NVS namespace setup** and basic parameter storage - ✅ Done
2. **Memory cache system** for live parameter updates - ✅ Done  
3. **Dual method API** (memory vs persist functions) - ✅ Done
4. **Integration with existing robot_config.h** structures - ✅ Done

#### Implementation Notes:
- **Files created:** `config_manager.h`, `config_manager.c`, `config_test.c`
- **System namespace fully implemented** with 10 configuration parameters
- **NVS operations** working with automatic error handling and wear leveling
- **Dual-method API** demonstrated: `config_set_*_memory()` vs `config_set_*_persist()`
- **Generic parameter API** foundation created for RPC integration
- **Factory reset** and migration infrastructure in place

### System Configuration Parameters (Implemented):
```c
typedef struct {
    // Safety settings
    bool emergency_stop_enabled;        // Emergency stop functionality
    uint32_t auto_disarm_timeout;       // Auto-disarm timeout (seconds) 
    float safety_voltage_min;           // Minimum battery voltage (volts)
    float temperature_limit_max;        // Maximum operating temperature (°C)
    uint32_t motion_timeout_ms;         // Motion command timeout
    uint32_t startup_delay_ms;          // Startup safety delay
    uint32_t max_control_frequency;     // Maximum control loop frequency (Hz)
    
    // System identification  
    char robot_id[32];                  // Unique robot identifier (from MAC)
    char robot_name[64];                // Human-readable robot name
    uint16_t config_version;            // Configuration schema version
} system_config_t;
```

### Usage Examples:
```c
// Initialize configuration system
config_manager_init();

// Get current configuration
const system_config_t* config = config_get_system();

// Live tuning (immediate effect, no flash write)
config_set_safety_voltage_min_memory(6.8f);
config_set_robot_name_memory("Test Robot");

// Check for unsaved changes
if (config_manager_has_dirty_data()) {
    // Save all changes at once
    config_manager_save_namespace(CONFIG_NS_SYSTEM);
}

// Or save immediately (live tuning + persistence)  
config_set_safety_voltage_min_persist(7.0f);

```

## Implementation Plan (Updated)

### Phase 1: Core Infrastructure ✅ **COMPLETED**

### Phase 2: RPC Layer
1. **JSON-based parameter API** over WebSocket/HTTP
2. **Generic parameter access** functions  
3. **Namespace save/reload** operations
4. **Configuration state management**

### Phase 3: Migration System  
1. **Version detection** and sequential migration framework
2. **Migration safety features** (backup/rollback)
3. **JSON export/import** system  
4. **External migration tools**

### Phase 4: Configurator Integration
1. **Web-based configurator portal** using RPC API
2. **Live parameter tuning** with immediate robot feedback
3. **Configuration backup/restore** functionality  
4. **Bulk configuration management** for multiple robots

## Summary

The NVS-based approach provides the optimal foundation for the hexapod's configuration system, offering superior performance for live parameter tuning while maintaining simplicity and robustness. The dual-method RPC design enables flexible configurator workflows, and the migration system ensures smooth firmware updates throughout development.