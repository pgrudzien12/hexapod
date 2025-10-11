# Bluetooth Classic SPP Controller Protocol

This document specifies the binary protocol used by the Bluetooth Classic controller driver for the hexapod locomotion framework.

## 1. Overview

The Bluetooth Classic controller uses the Serial Port Profile (SPP) to establish a wireless serial connection between a host device (phone, computer, etc.) and the hexapod. The protocol uses the same binary frame format as the WiFi TCP driver for consistency across transport layers.

## 2. Transport Layer

**Profile:** Bluetooth Serial Port Profile (SPP)  
**Mode:** Server (hexapod accepts incoming connections)  
**Security:** Legacy pairing with fixed PIN  
**Device Discovery:** General discoverable mode  

### 2.1 Pairing Strategy

The hexapod is designed for devices **without input capability** (no buttons, display, etc.):

- **Authentication Method:** Legacy PIN-based pairing
- **Default PIN:** `1234` (configurable at compile time)
- **SSP (Secure Simple Pairing):** Disabled by default
- **Device Name:** `HEXAPOD_BT` (configurable)
- **Auto-reconnect:** Supported after initial pairing

### 2.2 Connection Management

- **Server Name:** `HEXAPOD_SPP` (configurable)
- **Connection Timeout:** 1000ms (configurable)
- **Max Concurrent Connections:** 1
- **Watchdog:** Monitors frame reception and triggers failsafe on timeout

## 3. Frame Format

The protocol uses a fixed 74-byte binary frame format identical to the WiFi TCP protocol:

```
Offset | Size | Field        | Description
-------|------|--------------|------------------------------------------
0      | 2    | Sync         | 0xAA 0x55 (frame synchronization)
2      | 1    | Version      | 0x01 (protocol version)
3      | 1    | Flags        | Bit flags (currently unused, set to 0)
4      | 2    | Sequence     | uint16_t little-endian (frame counter)
6      | 2    | Payload Len  | uint16_t little-endian (expected: 64)
8      | 64   | Channels     | 32 × int16_t little-endian channels
72     | 2    | CRC16        | CRC16-CCITT over bytes 0-71
-------|------|--------------|------------------------------------------
Total: | 74   | Frame Size   |
```

### 3.1 Field Details

**Sync Pattern (0xAA 0x55):**
- Fixed 2-byte synchronization pattern for frame alignment
- Used to detect frame boundaries in the stream

**Version (0x01):**
- Protocol version identifier
- Frames with unsupported versions are discarded
- Current version: 1

**Flags (1 byte):**
- Reserved for future use
- Must be set to 0x00 in current version
- Will be used for features like acknowledgment requests, priority, etc.

**Sequence (uint16_t LE):**
- Frame sequence number for detecting dropped frames
- Incremented by sender for each frame
- Currently logged but not used for ordering

**Payload Length (uint16_t LE):**
- Expected value: 64 (32 channels × 2 bytes)
- Frames with incorrect payload length are discarded
- Allows for future protocol extensions

**Channels (32 × int16_t LE):**
- 32 controller channels in signed 16-bit format
- Range: -32768 to +32767 (full signed range)
- Little-endian byte order
- Maps to standard RC controls (see Channel Mapping section)

**CRC16-CCITT:**
- Polynomial: 0x1021
- Initial value: 0xFFFF
- Computed over header + payload (bytes 0-71)
- Frames with CRC mismatch are discarded

## 4. Channel Mapping

Channels are mapped to standard RC controls as follows:

| Channel | Control Function    | Range        | Notes                    |
|---------|-------------------|--------------|--------------------------|
| 0       | Right Stick Horiz | -32768..32767| Lateral body shift       |
| 1       | Left Stick Vert   | -32768..32767| Forward/backward speed   |
| 2       | Right Stick Vert  | -32768..32767| Body height adjustment   |
| 3       | Left Stick Horiz  | -32768..32767| Yaw rotation rate        |
| 4       | SWA (Arm)         | neg/pos      | Safety arm/disarm        |
| 5       | (Reserved)        | -            | Future use               |
| 6       | SWB (Pose Mode)   | neg/pos      | Walk/pose mode toggle    |
| 7       | SWC (Gait)        | thirds       | Gait selection (3-way)   |
| 8       | SWD (Terrain)     | neg/pos      | Normal/climb mode        |
| 9       | SRA (Knob)        | -32768..32767| Step frequency scaling   |
| 10-31   | (Unused)          | 0            | Reserved for future use  |

### 4.1 Stick Mapping
- **Negative values:** Left/Down/Back
- **Positive values:** Right/Up/Forward  
- **Zero:** Center/Neutral position
- **Deadband:** Applied in software (~4% around center)

### 4.2 Switch Mapping
- **Negative values (< 0):** Switch position 0/OFF
- **Positive values (> 0):** Switch position 1/ON

### 4.3 Gait Selection (Channel 7)
Three-way gait selection using channel value ranges:
- **-32768 to -10923:** Tripod gait (fast, dynamic)
- **-10922 to +10923:** Ripple gait (medium stability)
- **+10924 to +32767:** Wave gait (slow, maximum stability)

## 5. Error Handling

### 5.1 Frame Validation
1. **Sync Detection:** Search for 0xAA 0x55 pattern
2. **Version Check:** Discard unsupported versions
3. **Length Check:** Verify payload length = 64 bytes
4. **CRC Validation:** Compute and compare CRC16
5. **Range Check:** Channels automatically clamped to int16_t range

### 5.2 Connection Management
- **Initial Connection:** Device becomes discoverable, accepts pairing
- **Frame Timeout:** 1000ms without valid frame triggers failsafe
- **Reconnection:** Automatic after brief disconnection
- **Failsafe State:** All channels set to neutral/safe values

### 5.3 Pairing Process
1. Host scans for Bluetooth devices
2. Host selects "HEXAPOD_BT" from device list
3. Host enters PIN: `1234` (or configured PIN)
4. Hexapod automatically accepts with matching PIN
5. SPP connection established
6. Frame streaming begins immediately

## 6. Host Implementation Example

### 6.1 Frame Construction (C/C++)
```c
#include <stdint.h>

#define SYNC0 0xAA
#define SYNC1 0x55
#define VERSION 1
#define PAYLOAD_LEN 64

typedef struct {
    uint8_t sync[2];        // 0xAA, 0x55
    uint8_t version;        // 0x01
    uint8_t flags;          // 0x00
    uint16_t sequence;      // frame counter
    uint16_t payload_len;   // 64
    int16_t channels[32];   // channel data
    uint16_t crc;           // CRC16-CCITT
} __attribute__((packed)) bt_frame_t;

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; 
            else crc <<= 1;
        }
    }
    return crc;
}

void build_frame(bt_frame_t *frame, const int16_t *channels, uint16_t seq) {
    frame->sync[0] = SYNC0;
    frame->sync[1] = SYNC1;
    frame->version = VERSION;
    frame->flags = 0;
    frame->sequence = seq;
    frame->payload_len = PAYLOAD_LEN;
    
    for (int i = 0; i < 32; i++) {
        frame->channels[i] = channels[i];
    }
    
    frame->crc = crc16_ccitt((uint8_t*)frame, sizeof(*frame) - 2);
}
```

### 6.2 Android Bluetooth Example
```java
// Bluetooth connection setup
BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
BluetoothDevice device = adapter.getRemoteDevice("XX:XX:XX:XX:XX:XX");
BluetoothSocket socket = device.createRfcommSocketToServiceRecord(
    UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")); // SPP UUID

socket.connect();
OutputStream out = socket.getOutputStream();

// Frame transmission
byte[] frame = new byte[74];
// ... build frame ...
out.write(frame);
out.flush();
```

## 7. Configuration Options

The Bluetooth driver supports compile-time configuration:

```c
typedef struct {
    const char *device_name;           // "HEXAPOD_BT"
    const char *spp_server_name;       // "HEXAPOD_SPP"  
    bool enable_ssp;                   // false (no input capability)
    uint32_t pin_code;                 // 1234
    uint16_t connection_timeout_ms;    // 1000
} controller_bt_classic_cfg_t;
```

## 8. Security Considerations

- **PIN Security:** Change default PIN for production deployments
- **Range Limitation:** Bluetooth Classic ~10m range limits exposure
- **No Encryption:** Legacy pairing provides basic authentication only
- **Physical Access:** Device pairing requires physical proximity
- **Future:** Consider implementing application-layer authentication tokens

## 9. Troubleshooting

### 9.1 Pairing Issues
- **Wrong PIN:** Verify PIN matches configuration (default: 1234)
- **SSP Conflicts:** Ensure host doesn't require SSP when hexapod has it disabled
- **Discovery:** Check hexapod is in discoverable mode during pairing

### 9.2 Connection Issues  
- **Range:** Stay within ~10m of hexapod
- **Interference:** Minimize 2.4GHz interference (WiFi, other BT devices)
- **Power:** Ensure hexapod has sufficient power for BT radio

### 9.3 Protocol Issues
- **Frame Format:** Verify exact 74-byte frame structure
- **Byte Order:** Use little-endian for multi-byte fields
- **CRC:** Implement CRC16-CCITT correctly (poly 0x1021, init 0xFFFF)

## 10. Future Extensions

- **Bidirectional Protocol:** Status/telemetry frames from hexapod to host
- **Multiple Channels:** Support for additional sensor inputs
- **Authentication:** Application-layer token exchange
- **Compression:** Frame compression for higher update rates
- **BLE Variant:** Low-energy version for battery-powered hosts

---

This protocol provides a robust, low-latency control interface while accommodating the constraints of devices without input capability through the fixed PIN pairing strategy.