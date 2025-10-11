#ifndef CONTROLLER_INTERNAL_H
#define CONTROLLER_INTERNAL_H
// Internal helper API used by individual controller driver implementations.
// Not intended for inclusion outside controller driver source files.
#include <stdint.h>
#include <stdbool.h>
#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

bool controller_internal_lock(uint32_t timeout_ms);
void controller_internal_unlock(void);
void controller_internal_update_channels(const uint16_t *src);
void controller_internal_set_failsafe(void);
void controller_internal_set_connected(bool connected);
bool controller_internal_is_connected(void);
const controller_config_t *controller_internal_get_config(void);
const void *controller_internal_get_driver_cfg(size_t *out_size);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_INTERNAL_H
