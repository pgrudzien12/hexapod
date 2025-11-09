# Hexapod RPC System Design

## Overview

This document outlines the design for a Remote Procedure Call (RPC) system for the hexapod robot, inspired by Betaflight's MSP (MultiWii Serial Protocol) and CLI systems. The RPC system will provide real-time parameter tuning, robot control, and debugging capabilities.

## Goals

1. **Multi-transport support**: Bluetooth, WiFi, and Serial (UART)
2. **Non-blocking operation**: Run on separate CPU core to avoid interfering with main control loop
3. **Betaflight-inspired commands**: Simple, text-based commands for ease of use
4. **Configuration integration**: Seamless integration with the hybrid configuration system
5. **Robot control**: Direct joint and leg control for testing and debugging
6. **Performance**: Bulk operations for efficient data transfer

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        RPC System (Core 0)                     │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Bluetooth     │  │      WiFi       │  │     Serial      │ │
│  │   Transport     │  │   Transport     │  │   Transport     │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│           │                     │                     │         │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                Transport Abstraction Layer                  │ │
│  └─────────────────────────────────────────────────────────────┘ │
│           │                                                     │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                Command Parser & Dispatcher                  │ │
│  └─────────────────────────────────────────────────────────────┘ │
│           │                                                     │
│  ┌────────────────────┬────────────────────┬───────────────────┐ │
│  │   Config Commands  │  Control Commands  │  System Commands  │ │
│  └────────────────────┴────────────────────┴───────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
           │                     │                     │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ Configuration   │  │ Robot Control   │  │ System Status   │
│ Manager         │  │ (Legs/Joints)   │  │ & Diagnostics   │
│ (Core 1)        │  │ (Core 1)        │  │ (Core 1)        │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

## Transport Layer Design

### 1. Transport Abstraction

```c
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

### 2. Multi-Transport Manager

- **Concurrent listening**: All transports active simultaneously
- **Priority handling**: Bluetooth > WiFi > Serial (configurable)
- **Connection management**: Auto-detect active connections
- **Buffer management**: Per-transport receive/send buffers

### 3. Transport Implementations

#### Bluetooth Classic (Primary)
- **Protocol**: Bluetooth Serial Port Profile (SPP)
- **Connection**: Automatic pairing with known devices
- **Buffer size**: 256 bytes (typical BT classic limit)
- **Latency**: ~10-20ms (excellent for real-time tuning)

#### WiFi TCP (Secondary)
- **Protocol**: TCP socket server on configurable port (default: 5760)
- **Connection**: Multiple concurrent clients supported
- **Buffer size**: 1024 bytes (higher throughput)
- **Latency**: ~5-50ms depending on network

#### Serial UART (Fallback)
- **Protocol**: Standard UART (115200 baud default)
- **Connection**: Direct USB/UART connection
- **Buffer size**: 128 bytes
- **Latency**: ~1-5ms (lowest latency, highest reliability)

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

## Core Assignment Strategy

### Core 0 (Protocol Core)
- **RPC System**: Command parsing, response generation
- **Transport Management**: Bluetooth, WiFi, Serial handling  
- **Buffer Management**: Receive/transmit queues
- **Command Dispatch**: Route commands to Core 1 handlers

### Core 1 (Application Core)
- **Main Control Loop**: Robot locomotion and control
- **Configuration System**: NVS operations and parameter management
- **Robot Control**: Joint control, inverse kinematics, gait patterns
- **System Monitoring**: Status reporting, diagnostics

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
- **Stack optimization**: Minimize stack usage on both cores

## Implementation Phases

### Phase 1: Core Infrastructure
1. Transport abstraction layer
2. Command parser and dispatcher
3. Inter-core communication queues
4. Basic configuration commands (get/set)

### Phase 2: Configuration Integration  
1. Bulk configuration operations
2. Namespace enumeration
3. Parameter metadata access
4. Save/load operations

### Phase 3: Robot Control
1. Direct joint control commands
2. Leg positioning commands  
3. Safety interlocks and validation
4. Status reporting

### Phase 4: Advanced Features
1. Multiple transport support
2. Streaming telemetry
3. Authentication and security
4. Performance optimization

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