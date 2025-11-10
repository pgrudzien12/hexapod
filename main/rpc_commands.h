#pragma once
/*
 * RPC command parser with transport abstraction
 * Supports basic configuration inspection/modification and system operations.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "rpc_transport.h"

#ifdef __cplusplus
extern "C" { 
#endif

// Initialize RPC system (idempotent)
void rpc_init(void);

// Feed raw ASCII bytes (not null-terminated). Called from BT driver for non-binary frames.
void rpc_feed_bytes(const uint8_t *data, size_t len);

// Optional tick (not required now)
void rpc_poll(void);

// Callback used by RPC to send responses outward (must be set by transport)
typedef void (*rpc_send_fn_t)(const char *text);
void rpc_set_send_callback(rpc_send_fn_t fn);

#ifdef __cplusplus
}
#endif
