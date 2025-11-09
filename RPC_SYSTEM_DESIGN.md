# Hexapod RPC System Design

## Overview

This document outlines the design for a Remote Procedure Call (RPC) system for the hexapod robot, inspired by Betaflight's MSP (MultiWii Serial Protocol) and CLI systems. The RPC system will provide real-time parameter tuning, robot control, and debugging capabilities.

## Goals

1. **Bluetooth-first implementation**: Start with Bluetooth Classic for simplicity and debugging
2. **Single-core operation**: Run on same core as main control loop initially (Core 1)
3. **Betaflight-inspired commands**: Simple, text-based commands for ease of use
4. **Configuration integration**: Seamless integration with the hybrid configuration system
5. **Robot control**: Direct joint and leg control for testing and debugging
6. **Performance**: Bulk operations for efficient data transfer
7. **Future extensibility**: Design for easy addition of WiFi/Serial transports later

## Architecture Overview (Phase 1: Bluetooth-Only)

```
┌─────────────────────────────────────────────────────────────────┐
│                    RPC System (Core 1)                         │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐                                           │
│  │   Bluetooth     │                                           │
│  │   Classic SPP   │                                           │
│  └─────────────────┘                                           │
│           │                                                     │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                Command Parser & Dispatcher                  │ │
│  └─────────────────────────────────────────────────────────────┘ │
│           │                                                     │
│  ┌────────────────────┬────────────────────┬───────────────────┐ │
│  │   Config Commands  │  Control Commands  │  System Commands  │ │
│  └────────────────────┴────────────────────┴───────────────────┘ │
│                                │                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │ Configuration   │  │ Robot Control   │  │ System Status   │ │
│  │ Manager         │  │ (Legs/Joints)   │  │ & Diagnostics   │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Transport Layer Design (Phase 1: Bluetooth Only)

### 1. Bluetooth Classic Implementation

```c
// Simple Bluetooth RPC interface
typedef struct {
    bool is_connected;
    esp_spp_cb_param_t spp_param;
    uint32_t handle;
    char rx_buffer[256];
    size_t rx_len;
} rpc_bluetooth_t;

// Core functions
esp_err_t rpc_bluetooth_init(void);
bool rpc_bluetooth_is_connected(void);
int rpc_bluetooth_send(const char* data, size_t len);
int rpc_bluetooth_receive(char* buffer, size_t max_len);
```

### 2. Bluetooth Classic Configuration

- **Protocol**: Bluetooth Serial Port Profile (SPP)
- **Service Name**: "Hexapod RPC"
- **Device Name**: Configurable (default: "Hexapod-XXXXXX")
- **Connection**: Single client connection
- **Buffer size**: 256 bytes (SPP limit)
- **Latency**: ~10-20ms (excellent for real-time tuning)
- **Auto-reconnect**: Support automatic reconnection

### 3. Future Transport Abstraction (Phase 2+)

When adding WiFi/Serial support later, we'll implement:

```c
// Future transport abstraction (not implemented in Phase 1)
typedef enum {
    RPC_TRANSPORT_BLUETOOTH = 0,
    RPC_TRANSPORT_WIFI_TCP,
    RPC_TRANSPORT_SERIAL,
    RPC_TRANSPORT_COUNT
} rpc_transport_type_t;

typedef struct {
    rpc_transport_type_t type;
    bool (*is_connected)(void);
    int (*send)(const char* data, size_t len);
    int (*receive)(char* buffer, size_t max_len);
    esp_err_t (*init)(void);
    esp_err_t (*deinit)(void);
} rpc_transport_t;
```

## Command Protocol Design

### 1. Command Format (Betaflight-inspired)

```
<command> [<parameter1>] [<parameter2>] ... [<parameterN>]\r\n
```

**Examples:**
```
get system emergency_stop_enabled
set system emergency_stop_enabled true
save
status
leg 0 move 100 50 -20
joint 0 0 1500
```

### 2. Response Format

```
# Command successful
<command>: <result_data>
OK

# Command failed  
<command>: ERROR - <error_message>

# Multi-line response
<command>:
<line1>
<line2>
...
<lineN>
OK
```

### 3. Command Categories

#### Configuration Commands
```bash
# Individual parameter access
get <namespace> <parameter>           # Get single parameter
set <namespace> <parameter> <value>   # Set single parameter (memory)
setpersist <namespace> <parameter> <value>  # Set and save to NVS

# Bulk operations
export <namespace>                    # Get all parameters in a portable form
import <namespace> <data>             # Set all parameters from provided data

# Management
save [<namespace>]                    # Save namespace(s) to NVS
reload [<namespace>]                  # Reload namespace(s) from NVS
factory-reset                         # Factory reset (explicit destructive operation)
list namespaces                       # List all available namespaces
list <namespace>                      # List parameters in namespace
info <namespace> <parameter>          # Get parameter metadata
```

#### Robot Control Commands
```bash
# Joint control (servo-level)
joint <leg> <joint> <position>        # Set joint to PWM position (1000-2000)
jointangle <leg> <joint> <angle>      # Set joint to angle (degrees)

# Leg control (inverse kinematics)
leg <leg> move <x> <y> <z>           # Move leg tip to XYZ position (mm)
leg <leg> home                       # Move leg to home position
leg <leg> status                     # Get leg position and status

# Robot control
pose <roll> <pitch> <yaw> <x> <y> <z> # Set robot body pose
gait <type> <speed>                  # Start gait pattern
stop                                 # Emergency stop
home                                 # Move all legs to home position
```

#### System Commands
```bash
status                               # System status (voltages, temps, errors)
version                              # Firmware version and build info
reboot                              # Reboot system
cpu                                 # CPU usage and performance stats
memory                              # Memory usage statistics
tasks                               # FreeRTOS task information
```

## Configuration Integration

### 1. Hybrid Configuration API Usage

The RPC system will leverage our hybrid configuration system:

```c
// Individual parameter access (Approach B)
esp_err_t rpc_cmd_get(const char* namespace, const char* param) {
    // Use hexapod_config_get_* functions based on parameter type
    config_param_info_t info;
    hexapod_config_get_parameter_info(namespace, param, &info);
    
    switch(info.type) {
        case CONFIG_TYPE_BOOL:
            bool value;
            hexapod_config_get_bool(namespace, param, &value);
            rpc_send_response("get: %s", value ? "true" : "false");
            break;
        // ... other types
    }
}

// Bulk operations (Approach A) 
esp_err_t rpc_cmd_export_system() {
    const system_config_t* config = config_get_system();
    rpc_send_json_response(config);  // Serialize to JSON
    return ESP_OK;
}
```

### 2. Bulk Data Transfer Optimization

For efficient configurator operation:

```bash
# Fast system export (JSON format)
export system
# Response:
export system:
{
  "emergency_stop_enabled": true,
  "auto_disarm_timeout": 30,
  "safety_voltage_min": 6.5,
  ...
}
OK

# Fast bulk import
import system {"emergency_stop_enabled": false, "auto_disarm_timeout": 45}
# Response:
import system: 2 parameters updated
OK
```

## Core Assignment Strategy (Phase 1: Single Core)

### Core 1 (Main Application Core)
- **Main Control Loop**: Robot locomotion and control
- **RPC System**: Bluetooth handling, command parsing, response generation
- **Configuration System**: NVS operations and parameter management
- **Robot Control**: Joint control, inverse kinematics, gait patterns
- **System Monitoring**: Status reporting, diagnostics

### Integration Approach
- **Non-blocking RPC**: Process commands during main loop idle time
- **Priority**: Robot control takes priority over RPC commands
- **Safety**: Emergency stop and safety checks always active
- **Buffering**: Queue incoming commands, process during safe windows

### Future Multi-Core Support (Phase 2+)
When performance requires it, we can move RPC to Core 0:
- **Core 0**: RPC system, transport management, command parsing
- **Core 1**: Robot control, configuration, system monitoring
- **Inter-core communication**: FreeRTOS queues for command dispatch

## Performance Considerations

### 1. Latency Requirements
- **Configuration changes**: <50ms response time
- **Joint commands**: <10ms for real-time control
- **Status updates**: <100ms for monitoring

### 2. Throughput Optimization
- **Bulk transfers**: JSON format for configurator efficiency
- **Streaming data**: Optional high-frequency telemetry stream
- **Buffer sizing**: Optimized per-transport buffer sizes

### 3. Memory Management
- **Static allocation**: Avoid malloc/free in real-time paths
- **Buffer pools**: Pre-allocated command/response buffers
- **Stack optimization**: Minimize stack usage in single-core design
- **Shared resources**: Careful sharing between RPC and robot control

## Implementation Phases

### Phase 1: Bluetooth RPC Foundation
1. Bluetooth Classic SPP setup
2. Simple command parser and dispatcher
3. Basic configuration commands (get/set/export/import)
4. Single-core integration with main loop

### Phase 2: Configuration Integration  
1. Bulk configuration operations
2. Namespace enumeration and parameter discovery
3. Parameter metadata access and validation
4. Save/reload/factory-reset operations

### Phase 3: Robot Control Commands
1. Direct joint control commands
2. Leg positioning commands (inverse kinematics)
3. Safety interlocks and validation
4. System status reporting

### Phase 4: Advanced Features
1. Transport abstraction layer (add WiFi/Serial)
2. Multi-core implementation (move RPC to Core 0)
3. Streaming telemetry and real-time data
4. Authentication and security features

## Future Extensions

### 1. Configurator Integration
- **Web-based configurator**: Similar to Betaflight Configurator
- **Real-time tuning**: Live parameter adjustment
- **3D visualization**: Robot pose and leg positions

### 2. Advanced Control
- **Trajectory recording**: Record and playback movements
- **Gait optimization**: Automatic gait parameter tuning
- **Sensor integration**: IMU, touch sensors, etc.

### 3. Multi-Robot Support
- **Robot discovery**: Automatic robot detection on network
- **Swarm coordination**: Multi-robot command coordination
- **Load balancing**: Distribute commands across multiple robots

This design provides a solid foundation for a flexible, high-performance RPC system that can grow with the hexapod project's needs while maintaining compatibility with existing Betaflight-style tooling and workflows.