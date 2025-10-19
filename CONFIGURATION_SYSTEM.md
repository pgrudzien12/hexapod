# Hexapod Configuration System

## Overview

The hexapod robot uses a FAT filesystem-based configuration system for persistent storage of robot parameters. This system provides human-readable JSON configuration files that can be easily modified and backed up.

## System Architecture

### Partition Layout

The system uses a custom partition table optimized for 4MB flash with 1.5MB application space and 2.4MB FAT partition for configuration storage:

```csv
# Name,   Type, SubType, Offset,  Size,     Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x180000,
config,   data, fat,     0x190000,0x260000,
```

**Partition Sizes:**
- NVS: 24KB (for WiFi credentials, etc.)
- PHY Init: 4KB (RF calibration data)
- Factory App: 1.5MB (1536KB) - Main application code
- Config FAT: 2.4MB (2432KB) - Configuration files storage
- **Total Used: 3.94MB** (fits in 4MB flash with 64KB spare)

### Configuration Files Structure

The configuration is organized in a hierarchical JSON structure:

```
/config/
├── system.json          # Core system configuration
├── gpio_mapping.json    # GPIO pin assignments
├── mount_poses.json     # Robot stance and mount positions
├── leg_geometry.json    # Physical leg dimensions
└── legs/
    ├── leg_0.json       # Calibration data for leg 0
    ├── leg_1.json       # Calibration data for leg 1
    ├── leg_2.json       # Calibration data for leg 2
    ├── leg_3.json       # Calibration data for leg 3
    ├── leg_4.json       # Calibration data for leg 4
    └── leg_5.json       # Calibration data for leg 5
```

## ESP-IDF Partition Reference

The original partition definitions can be found in your ESP-IDF installation:
- **Location**: `C:\Users\user\esp\v5.4.2\esp-idf\components\partition_table\` (or similar depending on your ESP-IDF version)
- **Files**: Look for `partitions_*.csv` files for reference layouts
- **Note**: We use a "large" partition layout to accommodate the application code plus the additional 1MB FAT partition

## Implementation Details

### Key Components

1. **robot_config_fat.h/c**: Core FAT filesystem configuration management
2. **robot_config.h/c**: Main configuration API (maintains backward compatibility)
3. **partitions.csv**: Custom partition table definition
4. **sdkconfig**: ESP-IDF configuration with FAT filesystem enabled

### Configuration Categories

- **System Configuration**: Basic robot parameters, timings, and modes
- **GPIO Mapping**: Pin assignments for servos and sensors
- **Leg Geometry**: Physical dimensions and joint limits
- **Mount Poses**: Default stance and calibration positions
- **Per-Leg Calibration**: Individual servo calibration data

### API Functions

```c
// Core configuration management
esp_err_t robot_config_fat_init(void);
esp_err_t robot_config_fat_create_defaults(void);
esp_err_t robot_config_fat_load_all(void);
esp_err_t robot_config_fat_save_all(void);

// File system utilities
void robot_config_fat_print_file_tree(void);
void robot_config_fat_print_storage_info(void);
void robot_config_fat_list_files(void);
```

## Debugging and Data Access

### 1. Reading Configuration from Partition

To download and examine the configuration data from the ESP32:

```bash
# Read the entire config partition (2.4MB)
esptool.py --chip esp32 --port COM3 read_flash 0x190000 0x260000 config_partition.bin

# Mount the partition file on your computer (requires additional tools)
# On Windows: Use tools like OSFMount or similar to mount the FAT image
# On Linux: sudo mount -o loop config_partition.bin /mnt/config
```

### 2. Using ESP-IDF Monitor for Live Debugging

The system provides several runtime debugging commands accessible through the serial console:

```c
// Print current file tree with status
robot_config_fat_print_file_tree();

// Show storage usage information
robot_config_fat_print_storage_info();

// List all files with sizes
robot_config_fat_list_files();
```

### 3. File Access via ESP32 Code

```c
// Example: Read a configuration file directly
FILE* file = fopen("/config/system.json", "r");
if (file) {
    // Read and process file
    fclose(file);
}
```

### 4. Partition Analysis Tools

```bash
# Get partition information
idf.py partition-table

# Monitor real-time logs
idf.py monitor

# Flash only the partition table
idf.py partition-table-flash
```

### 5. Configuration Backup and Restore

**Backup Configuration:**
```bash
# Read the config partition
esptool.py --chip esp32 --port COM5 read_flash 0x190000 0x260000 hexapod_config_backup.bin
```

**Restore Configuration:**
```bash
# Write back the config partition
esptool.py --chip esp32 --port COM5 write_flash 0x190000 hexapod_config_backup.bin
```

## Configuration File Examples

### system.json
```json
{
  "stance_height": 120.0,
  "body_clearance": 80.0,
  "step_height": 40.0,
  "gait_period_ms": 2000,
  "controller_timeout_ms": 5000,
  "auto_save_interval_ms": 10000
}
```

### gpio_mapping.json
```json
{
  "leg_0": {
    "coxa_pin": 2,
    "femur_pin": 4,
    "tibia_pin": 16,
    "mcpwm_group": 0
  }
}
```

## Troubleshooting

### Common Issues

1. **Partition Not Found**: Ensure custom partition table is flashed
2. **Mount Failed**: Check that FAT filesystem is enabled in sdkconfig
3. **File Not Found**: Verify default configuration files are created
4. **Wear Leveling Errors**: Check partition size and alignment

### Recovery Procedures

1. **Reset to Defaults**:
   ```c
   robot_config_fat_create_defaults();
   ```

2. **Format Partition**:
   ```c
   esp_vfs_fat_spiflash_format_rw_wl("/config", "config");
   ```

3. **Complete Reflash**:
   ```bash
   idf.py erase-flash
   idf.py flash
   ```

## Performance Considerations

- **Startup Time**: FAT mounting adds ~100-200ms to boot time
- **Memory Usage**: ~32KB RAM for FAT filesystem buffers
- **Write Endurance**: Wear leveling extends partition lifespan
- **File Operations**: JSON parsing requires temporary heap allocation

## Migration Notes

This system replaces the previous NVS-based configuration storage while maintaining API compatibility. The main advantages are:

- Human-readable configuration files
- Easy backup and external editing
- Hierarchical organization
- Better debugging capabilities
- Room for future expansion

## Development Guidelines

1. **Always validate JSON syntax** before writing files
2. **Use the provided API functions** rather than direct file I/O
3. **Test configuration changes** with known-good defaults available
4. **Monitor storage usage** to prevent partition overflow
5. **Implement proper error handling** for all file operations