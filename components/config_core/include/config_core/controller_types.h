#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file controller_types.h
 * @brief Controller driver type definitions shared between config and main application
 * 
 * Extracted from main/controller.h to break circular dependency between
 * config components and main application code.
 */

/**
 * @brief Supported controller input driver types
 * 
 * Used in system configuration to persist controller driver selection.
 */
typedef enum {
    CONTROLLER_DRIVER_FLYSKY_IBUS = 0,  ///< FlySky UART iBUS protocol (current default)
    CONTROLLER_DRIVER_UART_GENERIC,     ///< Generic raw UART protocol (placeholder)
    CONTROLLER_DRIVER_BT_CLASSIC,       ///< Bluetooth Classic SPP
    CONTROLLER_DRIVER_BT_LE,            ///< Bluetooth Low Energy (placeholder)
    CONTROLLER_DRIVER_WIFI_TCP,         ///< WiFi TCP binary protocol
    CONTROLLER_DRIVER_COUNT             ///< Number of driver types (for validation)
} controller_driver_type_e;

#ifdef __cplusplus
}
#endif
