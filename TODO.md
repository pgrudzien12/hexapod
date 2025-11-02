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

## 📋 Implementation Priority

### Phase 1: Foundation (High Priority)
1. FlySky controller debugging and reconnection
2. Electronics/connector improvements
3. NVS storage implementation for configuration persistence
4. Error handling and safety mechanisms

### Phase 2: Enhanced Control (Medium Priority)
1. IMU integration and terrain leveling
2. Touch sensors implementation
3. Advanced motion modes (Fast/Emergency)
4. Swing trajectory improvements

### Phase 3: Advanced Features (Lower Priority)
1. ESP-NOW controller development
2. Jetson Nano integration
3. Dual camera system
4. Computer vision capabilities
5. Advanced sensor fusion

---

## 📝 Notes
- Current system has comprehensive KPP (Kinematic Path Planning) implementation
- Debug infrastructure is in place for development and tuning
- Motion limiting and safety systems are partially implemented
- FlySky controller was previously working but needs debugging for signal stability

---

*Last updated: November 2, 2025*
