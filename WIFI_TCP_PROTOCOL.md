# WiFi TCP Controller Protocol (DEPRECATED)

**This binary protocol is deprecated and no longer in use.**

The WiFi TCP controller now communicates with the hexapod locomotion framework using the [RPC System](./RPC_SYSTEM_DESIGN.md). Control data is sent as a text-based command.

## New RPC-Based Protocol

Instead of a binary frame, the controller sends a command string to the RPC system over the TCP connection.

**Command:** `set controller <ch0> <ch1> ... <ch31>`

- `<chN>` is a signed 16-bit integer representing the value for channel N.

### Example
`set controller 1500 -200 0 0 1000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0`

This command updates the 32 controller channels, which are then processed by the robot's control system. This approach unifies all input methods (WiFi, Bluetooth, etc.) under a single, extensible RPC interface.

For more details on the RPC system, please refer to the [RPC User Guide](./RPC_USER_GUIDE.md).
