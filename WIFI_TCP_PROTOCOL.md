# WiFi TCP Controller Protocol (Version 1)

This document specifies the binary protocol used by the WiFi TCP controller driver (`controller_wifi_tcp.c`). It enables a host application (e.g. configuration portal, joystick bridge, or scripting tool) to stream up to 32 signed channels to the robot at low latency.

## Goals
- Low overhead, fixed-size primary frame (predictable parsing, minimal allocations)
- Versioned for forward compatibility
- Integrity checking (CRC16-CCITT)
- Supports full signed 16-bit range per channel (-32768..32767)
- Extensible flags field for future feature negotiation

## Frame Layout (Version 1)
```
Offset  Size  Field                Notes
0       2     Sync (0xAA 0x55)     Magic header for alignment
2       1     Version (0x01)       Increment on incompatible changes
3       1     Flags                Bit0: channels present (must be 1 for v1)
4       2     Sequence (LE)        Monotonic per-sender, rollover allowed
6       2     Payload Length (LE)  For v1 must be 64 (32 * int16)
8       64    Channels             32 * int16 little-endian, signed
72      2     CRC16-CCITT          Poly 0x1021, init 0xFFFF, over bytes 0..71
Total: 74 bytes
```

### Endianness
All multi-byte integers are little-endian (consistent with ESP32 + most desktop architectures). Network stack does not perform hton/ntoh on content fields except TCP/IP headers.

### Channels
- Each channel: signed 16-bit, full scale -32768..32767.
- Drivers normalize internally to [-1,1] using `value / 32767.0f`.
- Unused channels should be set to 0 (neutral) by the sender.

### Sequence Field
- Optional for receiver; used to detect out-of-order or dropped frames.
- 16-bit rollover is acceptable; no explicit wrapping notification.

### Flags (v1)
Bit | Meaning
----|--------
0   | Channels present (must be 1)
1..7| Reserved (set to 0)

Future frames could use additional flags to indicate optional metadata appended after channel payload (TLVs), variable channel count, or compression.

### CRC16 Calculation
- Polynomial: 0x1021
- Initial value: 0xFFFF
- No reflection (input nor output) and no final XOR.
- Implemented in driver as iterative shift/XOR.

Pseudocode:
```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i=0;i<len;++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b=0;b<8;++b) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}
```
Compute over bytes 0..71 (header + channels) and append little-endian CRC.

## Parsing Algorithm (Receiver)
1. Read 8-byte header.
2. Validate sync bytes (AA 55).
3. Check version (==1); else drop connection or ignore frame.
4. Check flags bit0 set.
5. Validate payload length (==64).
6. Read 64-byte channel payload.
7. Read 2-byte CRC.
8. Compute CRC16 over header + payload and compare; if mismatch discard frame.
9. Deserialize 32 little-endian int16 values; store directly.
10. Update shared channel buffer; mark connection alive.

## Normalization on Embedded Side
Inside `controller_decode`:
```
float norm = (float)raw / 32767.0f; // saturates outside range
```
Deadband is then applied, mapping to locomotion commands.

## Timing & Rate
- Recommended frame rate: 50–150 Hz.
- Timeout: 1 second without valid frame triggers failsafe (all zeros).

## Example Host Implementation (Python)
```python
import socket, struct
from itertools import count

HOST = '192.168.4.1'  # ESP32 AP or LAN address
PORT = 5555

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

with socket.create_connection((HOST, PORT)) as s:
    for seq in count():
        channels = [0]*32
        # Example: sweep channel 0
        angle = (seq % 200) - 100
        channels[0] = int(max(-32768, min(32767, angle * 300)))
        payload = struct.pack('<32h', *channels)
        header = struct.pack('<2B2B2H', 0xAA, 0x55, 0x01, 0x01, seq & 0xFFFF, len(payload))
        frame_wo_crc = header + payload
        crc = crc16_ccitt(frame_wo_crc).to_bytes(2, 'little')
        frame = frame_wo_crc + crc
        s.sendall(frame)
```

## Error Handling Recommendations (Host)
- On socket error, reconnect with exponential backoff.
- Keep last sent frame cached to re-transmit quickly after reconnect.
- Optional: send heartbeat frames even if channels unchanged to avoid timeout.

## Future Extensions (Version Negotiation)
Potential enhancements for version 2:
- Variable channel count (payload length differing from 64)
- Optional metadata TLVs (e.g., calibration data, latency stats)
- Compression flag (delta frames)
- Authentication / HMAC field

## Security Considerations
Current protocol is unauthenticated. On untrusted networks, consider:
- Running in a closed AP mode
- Adding an application-layer challenge/response before accepting frames
- Migrating to TLS or DTLS (higher cost) for encrypted integrity

## Interoperability Checklist
- [ ] Sync bytes correct
- [ ] Version = 1
- [ ] Flags bit0 set
- [ ] Payload length = 64
- [ ] CRC correct
- [ ] 32 * int16 values little-endian

If all above pass and you still have no motion, verify receiver selected the WiFi driver and that failsafe cleared (connection logs show "client connected").

---
For issues or proposed extensions, open a PR or issue referencing this spec.
