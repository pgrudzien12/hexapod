# Hexapod RPC User Guide

## Overview

The Hexapod robot includes a Remote Procedure Call (RPC) system that allows real-time configuration and control via Bluetooth Classic connection. This system enables parameter tuning, configuration management, and system monitoring without requiring firmware recompilation.

## Connection Setup

### Prerequisites
- Hexapod robot powered on and running
- Computer or mobile device with Bluetooth Classic support
- Serial terminal application (PuTTY, Tera Term, RealTerm, or similar)

### Pairing Process

1. **Power On Robot**: Ensure the hexapod is powered and the main application is running
2. **Bluetooth Discovery**: The robot advertises as "HEXAPOD_XXXXXX" where XXXXXX is derived from the MAC address
3. **Pairing**: 
   - Scan for Bluetooth devices and select the hexapod
   - When prompted for PIN, enter: **1234** (default)
   - Windows will create virtual COM ports for the connection
4. **Port Selection**: Use the "Outgoing" COM port (not "Incoming")

### Terminal Configuration


Version: 1.0

Purpose: This document specifies the Hexapod RPC protocol for a client implementer. It is language-agnostic and focuses on behavior, framing, response semantics, error handling, and robust client recommendations suitable for a Python client.

1. Transport and Connection

- Phase 1 uses Bluetooth Classic SPP. The robot advertises a device name with a fixed prefix plus a MAC-derived suffix (for example, "HEXAPOD_XXXXXX"). Pairing requires a PIN (default 1234). The SPP link provides a bidirectional byte stream.

- Clients should open the platform-provided serial endpoint (virtual COM port on Windows, /dev/tty.* on Unix) or use a native SPP socket where available. The logical transport is an RFCOMM byte stream. Clients must treat the link as an unreliable serial channel.

- Only one SPP client is accepted at a time.

2. Line Framing and Encoding

- Commands are ASCII text terminated by LF, CR, or CRLF. Clients should normalize outgoing line endings to LF and trim trailing whitespace on incoming lines.

- Maximum command line length is 255 printable bytes. Longer lines are rejected with an "ERROR line too long" response. Implement client-side checks to prevent sending overlong lines.

- Responses are line-oriented and terminated with CRLF. For multi-line responses (e.g., JSON export), the device emits the payload lines and then a single "OK" line to mark completion. Clients should accumulate lines until the terminating "OK".

- Use ASCII/UTF-8 for text; avoid non-ASCII unless specific parameters explicitly support it.

3. Command Syntax and Semantics

- Commands are space-separated tokens: a verb followed by arguments. No quoting or escaping is implemented in Phase 1; string values must not contain spaces.

- Supported verbs (Phase 1): help, version, list, get, set, setpersist, save, export, factory-reset.

- Examples of semantics: "get <ns> <param>" returns the parameter value; "set <ns> <param> <value>" changes it in memory; "setpersist" changes and writes to NVS immediately.

4. Response Patterns

- Normal success: concise response text followed by an "OK" line. Single-line queries may return the data and then OK.

- Errors return a short message that begins with the command or the token "ERROR" and a brief explanation. Clients should treat any non-OK terminator as an error and surface the message to user code.

- For JSON export, the client must collect all payload lines until the final "OK" then join them into a complete JSON document before parsing.

5. Parameter Types and Value Formats

- Parameter types exposed by the server include boolean, integer, unsigned integer, float, and string. Use the following textual representations: booleans as "true"/"false" (or "1"/"0"), integers as decimal text, floats as decimal with optional fraction, strings as plain text without spaces.

- The server performs validation and returns errors if values are out-of-range or invalid. Clients should handle and propagate these errors rather than attempting to guess parameter bounds.

6. Typical Client Workflows

- Single-parameter read: send get command, wait for value line and OK.

- Batch change and persist: send multiple set commands locally, then send save to persist all changes in the namespace.

- Atomic persist: use setpersist for single-parameter set-and-save operations.

- Export and parse: send export, accumulate payload until OK, parse JSON.

- Factory reset: request explicit confirmation from the user, then send factory-reset and confirm success.

7. Robustness and Error Handling

- Timeouts: apply conservative timeouts (0.5–2s) for normal commands; allow longer timeouts for JSON export or reboot operations.

- Reconnect strategy: implement exponential backoff on disconnect, and re-synchronize by issuing a help or version command on reconnect.

- Partial responses: on malformed or truncated responses, discard up to the next terminator and optionally retry the command once.

8. Concurrency and Rate Control

- The device processes RPC on the main control loop while maintaining robot control priority. Avoid continuous high-rate command streams. For control commands, limit to tens of commands per second; configuration operations should be much less frequent.

- If you require transactional changes, prefer set+save or sequential setpersist calls spaced with short delays.

9. Export Format (JSON)

- The export output is a JSON object with parameter names as keys and JSON-native types as values. Clients must support parsing JSON that arrives in multiple chunks and validate parsed values before applying them.

10. Client Implementation Checklist (Python-specific guidance)

- Connectivity: wrap the serial/SPP connection in a class that provides send_line() and receive_line() primitives, handles line termination normalization, and exposes connection status.

- Request/Response model: implement a synchronous request/response method that sends a line and waits for a response plus OK. Use per-request timeouts and a simple retry policy.

- Export handling: support streaming reads and incremental JSON assembly to avoid large temporary buffers in constrained environments.

- API surface: implement a minimal client API for common operations (list_namespaces, list_parameters, get, set, setpersist, save, export, factory_reset, version, help).

- Safety: add an explicit confirmation mechanism for destructive calls (factory-reset) in any interactive or automated client.

- Tests: include automated tests that exercise connect/disconnect, get/set persistence, export parse, and factory-reset behavior.

11. Testing Strategy

- Manual verification: use an interactive terminal to test commands and persistence across reboots.

- Automated scenario: script a test that connects, reads version, exports config, toggles a parameter, saves, reboots the device, and verifies persistence.

- Edge cases: test overlong lines, invalid tokens, malformed JSON, and rapid-firing commands.

12. Extensibility and Future Compatibility

- Abstract transport: design the client so the transport is pluggable (Bluetooth SPP, TCP socket, plain serial). Keep framing and parsing code separate from the transport implementation.

- Metadata and import: when the server adds parameter metadata or an import command, update the client to perform schema-aware validation and atomic bulk imports.

13. Security and Safety

- Treat SPP as a semi-trusted link during Phase 1. Use pairing and device-level protections. Require explicit confirmation for destructive operations.

14. Glossary

- SPP: Serial Port Profile (Bluetooth Classic)  
- RPC: Text-based Remote Procedure Call  
- NVS: Non-volatile storage used by the device for persistent configuration

---

If you want I can now produce a Python client skeleton and unit-test outline that follows this spec (no device dependencies). That skeleton will include a small transport abstraction and examples of the request/response loop described above.
```

### Persistent Configuration Changes  
```
setpersist system emergency_stop_enabled true
setpersist system auto_disarm_timeout 45
```

### Configuration Backup and Management
```
export system
save system
factory-reset
```

## Error Messages and Troubleshooting

### Connection Issues
- **No response**: Verify correct COM port selection (use Outgoing port)
- **Garbled text**: Check terminal encoding (use ASCII or UTF-8)
- **Connection drops**: Ensure stable Bluetooth range and power

### Command Errors
- **unknown: [command]** - Command not recognized, try "help"
- **usage: [format]** - Incorrect command syntax, check parameters
- **get: not found** - Parameter name doesn't exist in namespace
- **set: not found** - Parameter name doesn't exist for setting
- **ERROR line too long** - Command exceeds 255 character limit
- **ERROR [operation]** - General operation failure, check parameter values

### Parameter Errors
- **ERROR get [type]** - Failed to retrieve parameter value
- **ERROR set [parameter]** - Failed to set parameter (check value format)
- **ERROR type** - Unsupported parameter type for operation

## Limitations and Notes

### Current Limitations
- Only system namespace implemented (more namespaces planned)
- String parameters cannot contain spaces
- Import functionality not yet implemented (export only)
- Single concurrent Bluetooth connection
- No command history or auto-completion

### Safety Considerations
- Changes to safety parameters affect robot operation immediately
- Always verify safety_voltage_min and temperature_limit_max values
- Use factory-reset cautiously as it erases all custom settings
- Emergency stop settings should be verified after any changes

### Performance Notes
- Commands process during main loop idle time
- Robot control takes priority over RPC commands
- Large exports may take several seconds to complete
- Bluetooth latency typically 10-20ms for parameter changes

## Advanced Usage

### Configuration Workflows

**Development Tuning**
1. Use "set" commands for rapid testing
2. Verify behavior with multiple "get" commands  
3. Use "setpersist" or "save" when satisfied with values

**Production Deployment**
1. Export current configuration for backup
2. Apply required changes with "setpersist"
3. Verify with "export" and testing
4. Document changes for future reference

**Emergency Recovery**
1. Connect via Bluetooth RPC
2. Use "factory-reset" to restore known good state
3. Reapply critical settings with "setpersist"
4. Test basic functionality before full operation

## Future Enhancements

Planned features for future releases:
- Import command for bulk configuration loading
- Additional configuration namespaces (joint calibration, motion limits)
- String parameter quoting for spaces
- Real-time telemetry streaming  
- WiFi and Serial transport options
- Command history and tab completion
- Multi-robot discovery and management

## Support

For technical support or feature requests, refer to the project documentation or contact the development team. Always include the firmware version (from "version" command) when reporting issues.