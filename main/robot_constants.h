#pragma once

/**
 * @file robot_constants.h
 * @brief Fundamental robot configuration constants
 * 
 * This file defines the core physical constants that define the robot's
 * structure. These should only be changed if the physical robot design changes.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Robot physical structure
#define NUM_LEGS 6              // Hexapod has 6 legs
#define NUM_JOINTS_PER_LEG 3    // Each leg has 3 joints (coxa, femur, tibia)

#ifdef __cplusplus
}
#endif