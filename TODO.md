# Hexapod Robot Project TODO List

## 🎯 Project Vision & New Goals

### Hardware Improvements
- [ ] **Touch sensors at each leg**
  - Research suitable force/pressure sensors for leg-ground contact detection
  - Design mounting system for sensors on leg tips
  - Implement sensor reading and processing
  - Integrate touch feedback into gait control system

- [ ] **Electronics & Power System Upgrade**
  - Replace/improve current power board to ESP32 connectors
  - Design more reliable electrical connections
  - Consider PCB design for better signal integrity
  - Implement proper power management and distribution

- [ ] **IMU Integration for Terrain Leveling**
  - Add IMU sensor (accelerometer + gyroscope)
  - Implement body position/orientation estimation
  - Develop terrain leveling algorithms
  - Integrate IMU feedback into whole body control system

### Controller Systems
- [ ] **FlySky Controller Reconnection**
  - Debug signal stability issues caused by electronics interference
  - Implement proper signal filtering/shielding
  - Test and validate reliable communication
  - Add failsafe mechanisms

- [ ] **ESP-NOW Controller Implementation**
  - Design ESP-NOW communication protocol
  - Implement controller device (separate ESP32)
  - Add wireless communication handling
  - Create user interface for ESP-NOW controller

### Vision & Processing
- [ ] **Jetson Nano Integration**
  - Add Jetson Nano compute module to the system
  - Design mounting and power solution
  - Implement dual camera system
  - Develop computer vision capabilities
  - Create communication bridge between ESP32 and Jetson Nano

### Configuration & Tuning Portal
- [ ] **Hexapod Configurator (Betaflight-inspired)**
  - Design web-based configuration portal for robot tuning
  - Implement RPC-like communication protocol between configurator and ESP32
  - Create real-time parameter adjustment interface
  - Add live telemetry and monitoring capabilities
  - Implement configuration backup/restore functionality

---

## 🔧 Existing Code TODOs

### Configuration & Storage
- [ ] **NVS Storage Implementation** (`kpp_config.h`, `robot_config.h`)
  - Add NVS storage for runtime tuning parameters
  - Persist robot configuration fields in NVS
  - Load configuration values at boot
  - Add NVS storage keys for runtime parameter tuning

- [ ] **Per-leg Configuration** (`robot_config.c`)
  - Replace hardcoded values with storage-loaded values per leg
  - Consider per-leg geometry differences (mirrors, tolerances)
  - Add leg-specific calibration data storage

### Motion Control System
- [ ] **Advanced Motion Modes** (`kpp_system.h`)
  - Add MOTION_MODE_FAST for quick reactions
  - Add MOTION_MODE_EMERGENCY for fall recovery
  - Add mode_transition_time for multi-mode support
  - Implement fast mode limits
  - Implement emergency mode limits

- [ ] **Swing Trajectory Improvements** (`swing_trajectory.c`, `swing_trajectory.h`)
  - Make swing trajectory parameters configurable via Kconfig or runtime config
  - Make max turn angle configurable (currently hardcoded to 0.4 rad)
  - Make turn direction configurable
  - Add simple body roll/pitch offsets when pose_mode is active
  - Add yaw (wz) coupling to bias per-leg x/y for turning
  - Consider exposing trajectory parameters via config or calibration struct

- [ ] **Gait Scheduler Enhancements** (`gait_scheduler.c`)
  - Consider separate forward/turning components and modulate phase rate
  - Replace current groupings with well-defined groupings and exact phase windows per gait

### Control & Safety
- [ ] **Error Handling & Safety** (`whole_body_control.c`)
  - Add error signaling so robot_control can engage a safe stop
  - Implement comprehensive safety mechanisms
  - Add system health monitoring

- [ ] **Controller Configuration** (`controller.c`)
  - Make DEAD_BAND configurable via Kconfig or runtime configuration
  - Add controller parameter tuning interfaces

### Future Sensor Integration
- [ ] **Coordinate Frame Transformations** (`kpp_config.h`)
  - Add coordinate frame transformation parameters
  - Implement sensor fusion framework

- [ ] **Sensor Fusion Parameters** (`kpp_config.h`)
  - Design sensor fusion architecture for IMU/force sensors
  - Implement sensor data processing pipeline
  - Add sensor calibration procedures

---

## 🖥️ Hexapod Configurator Portal (Betaflight-inspired)

### Core Infrastructure
- [ ] **Communication Protocol**
  - Design JSON-RPC or MessagePack-based protocol for ESP32 ↔ Configurator
  - Implement WebSocket connection for real-time communication
  - Add command/response handling with error codes
  - Create parameter read/write API endpoints
  - Implement telemetry streaming protocol

- [ ] **Web Application Framework**
  - Create React/Vue.js based configurator interface
  - Design responsive UI for desktop and tablet use
  - Implement WebSocket client for ESP32 communication
  - Add configuration state management
  - Create reusable UI components for parameter tuning

### Configuration Pages & Features

#### 🦿 **Leg Configuration Tab**
- [ ] Individual leg parameter tuning
  - Per-leg servo angle offsets and limits
  - Leg geometry parameters (coxa, femur, tibia lengths)
  - Servo direction and calibration settings
  - Real-time leg position visualization (3D model)
  - "Test Leg Movement" controls for each leg

#### ⚙️ **Motion Control Tab**
- [ ] Gait pattern configuration
  - Gait type selection (tripod, wave, ripple, custom)
  - Step height, length, and timing parameters
  - Swing trajectory curve parameters
  - Body height and stance width settings
  - Real-time gait visualization

#### 🎮 **Controller Settings Tab**
- [ ] Input device configuration
  - FlySky controller channel mapping and calibration
  - ESP-NOW controller pairing and setup
  - Dead zones and sensitivity curves
  - Control mode switching (manual/autonomous)
  - Input signal monitoring and diagnostics

#### 🧭 **Sensors & IMU Tab**
- [ ] Sensor calibration and monitoring
  - IMU calibration wizard (accelerometer, gyroscope)
  - Touch sensor threshold settings per leg
  - Real-time sensor data visualization
  - Terrain leveling algorithm parameters
  - Sensor fusion weight settings

#### 🔧 **System Parameters Tab**
- [ ] Core system configuration
  - KPP motion limits (velocity, acceleration, jerk)
  - Safety parameters and emergency stops
  - Power management settings
  - Debug logging levels
  - System health monitoring

#### 📊 **Telemetry & Monitoring Tab**
- [ ] Real-time robot status
  - Live 3D robot visualization showing current pose
  - Servo positions and load monitoring
  - Battery voltage and current consumption
  - System temperature monitoring
  - Error logs and diagnostic information

#### 💾 **Configuration Management Tab**
- [ ] Profile and backup management
  - Save/load configuration profiles
  - Export/import settings to/from files
  - Factory reset functionality
  - Configuration versioning and diff viewing
  - Automatic backup before major changes

### Advanced Features
- [ ] **Live Tuning & Testing**
  - Real-time parameter adjustment with immediate effect
  - "Safe Mode" with automatic revert after timeout
  - Parameter validation and bounds checking
  - Undo/redo functionality for parameter changes
  - Batch parameter updates

- [ ] **Calibration Wizards**
  - Automated servo calibration sequence
  - IMU calibration step-by-step guide
  - Leg geometry measurement assistant
  - Gait optimization wizard
  - Touch sensor threshold auto-calibration

- [ ] **Diagnostic Tools**
  - Servo health check and performance testing
  - Communication link quality monitoring
  - System performance profiling
  - Error code lookup and troubleshooting guide
  - Log file viewer and analysis

### ESP32 Firmware Support
- [ ] **Parameter Storage System**
  - Extend NVS storage to support all configurable parameters
  - Implement parameter validation and bounds checking
  - Add parameter change notification system
  - Create parameter schema/metadata system
  - Implement atomic parameter updates

- [ ] **Web Server & API**
  - Implement HTTP server for configurator hosting
  - Add WebSocket server for real-time communication
  - Create REST API for parameter access
  - Implement file upload/download for configurations
  - Add OTA update support through configurator

- [ ] **Telemetry System**
  - Real-time data streaming to configurator
  - Configurable telemetry rates and data selection
  - Data logging to SD card or flash memory
  - Remote logging capabilities
  - Performance metrics collection

---

## 📋 Implementation Priority

### Phase 1: Foundation (High Priority)
1. FlySky controller debugging and reconnection
2. Electronics/connector improvements
3. NVS storage implementation for configuration persistence
4. Error handling and safety mechanisms
5. **Basic configurator infrastructure** (communication protocol, web framework)

### Phase 2: Enhanced Control & Configuration (Medium Priority)
1. **Core configurator pages** (Leg Config, Motion Control, System Parameters)
2. IMU integration and terrain leveling
3. Touch sensors implementation
4. Advanced motion modes (Fast/Emergency)
5. Swing trajectory improvements
6. **Configuration management and calibration wizards**

### Phase 3: Advanced Features (Lower Priority)
1. **Advanced configurator features** (telemetry, diagnostics, live tuning)
2. ESP-NOW controller development
3. Jetson Nano integration
4. Dual camera system
5. Computer vision capabilities
6. Advanced sensor fusion

---

## 📝 Notes
- Current system has comprehensive KPP (Kinematic Path Planning) implementation
- Debug infrastructure is in place for development and tuning
- Motion limiting and safety systems are partially implemented
- FlySky controller was previously working but needs debugging for signal stability

---

*Last updated: November 2, 2025*
