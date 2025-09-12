#ifndef TRACKING_H
#define TRACKING_H

#include "types.h"

/**
 * @brief Initialize solar tracking system
 * 
 * Sets up GPS location, initializes panel position tracking, and prepares
 * system for first sun acquisition. Must be called after all hardware
 * components are initialized.
 * 
 * @note Does not move motors - call tracking_first_run() to begin tracking
 */
void tracking_init(void);

/**
 * @brief Perform initial sun acquisition and calibration
 * 
 * Gets fresh GPS fix, calculates current sun position, and moves panel
 * to track sun for the first time. Also performs reference position
 * calibration if needed.
 * 
 * @note Should be called once after system startup
 * @note Will move panel - ensure clear mechanical path
 */
void tracking_first_run(void);

/**
 * @brief Main solar tracking loop with smart power management
 * 
 * Continuously tracks sun position with periodic GPS updates and battery
 * monitoring. Automatically exits when sun sets, battery low, or other
 * conditions require shutdown.
 * 
 * Loop behavior:
 * - Updates GPS location every 10 cycles (reduces power)
 * - Calculates current sun position every UPDATE_INTERVAL_SEC
 * - Moves panel only if movement exceeds threshold
 * - Monitors battery and weather conditions
 * 
 * @note Blocking function - runs until tracking should stop
 */
void tracking_loop(void);

/**
 * @brief Prepare system for sleep/shutdown
 * 
 * Moves panel to safe storage position, stops all motors, and optionally
 * enters deep sleep mode until next sunrise. Called when tracking ends.
 * 
 * @note System will wake automatically or require manual restart
 */
void tracking_prepare_sleep(void);

#endif