#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "types.h"
#include <stdbool.h>

/**
 * @brief Initialize linear actuator motor control system
 * 
 * Configures PWM and direction pins for dual-axis solar tracking using:
 * - 2x 12V linear actuators (200mm stroke, 220lbs force)
 * - Cytron MDD20A dual motor driver (20A, 6-30V)
 * - ESP32 PWM channels for speed control
 * 
 * Hardware Configuration:
 * - Elevation actuator: GPIO25 (PWM), GPIO26 (DIR)
 * - Azimuth actuator: GPIO27 (PWM), GPIO14 (DIR)
 * - PWM frequency: 1kHz (optimal for motor drivers)
 * - Resolution: 10-bit (0-1023 duty cycle range)
 * 
 * @note Must be called before any motor movement functions
 */
void motor_init(void);

/**
 * @brief Move azimuth actuator to track sun's horizontal position
 * 
 * Controls horizontal (left/right) solar panel positioning for sun tracking.
 * Positive angles move panel west (afternoon sun), negative moves east (morning).
 * 
 * @param target_degrees Target azimuth angle relative to reference position
 * @param speed_percent Motor speed as percentage (10-80% recommended)
 * 
 * @note Blocks until movement complete or timeout occurs
 * @note Movement time calculated based on actuator speed (11.938mm/sec)
 */
void motor_move_azimuth_degrees(float target_degrees, int speed_percent);

/**
 * @brief Move elevation actuator to track sun's vertical position  
 * 
 * Controls vertical (up/down) solar panel positioning for sun tracking.
 * Positive angles tilt panel up (high sun), negative tilts down (low sun).
 * 
 * @param target_degrees Target elevation angle relative to reference position
 * @param speed_percent Motor speed as percentage (10-80% recommended)
 * 
 * @note Blocks until movement complete or timeout occurs
 * @note Elevation range typically 0° (horizontal) to 90° (vertical)
 */
void motor_move_elevation_degrees(float target_degrees, int speed_percent);

/**
 * @brief Move both axes simultaneously to sun position
 * 
 * Efficient dual-axis positioning for solar tracking. Moves both actuators
 * in parallel to minimize tracking time and power consumption.
 * 
 * @param azimuth_degrees Target horizontal angle (east/west positioning)
 * @param elevation_degrees Target vertical angle (up/down positioning)  
 * @param speed_percent Motor speed percentage for both axes
 * 
 * @note Completes when slower axis finishes movement
 * @note Recommended for normal sun tracking operations
 */
void motor_move_to_sun_position(float azimuth_degrees, float elevation_degrees, int speed_percent);

/**
 * @brief Emergency stop all motor movement
 * 
 * Immediately stops both actuators and disables PWM output.
 * Used for safety shutdowns, limit switch triggers, or system errors.
 * 
 * @note Non-blocking function - returns immediately
 * @note Motors may coast briefly due to inertia
 */
void motor_stop_all(void);

/**
 * @brief Move panel to safe storage position
 * 
 * Positions panel for overnight storage, storm protection, or maintenance.
 * Typical safe position: horizontal (0° elevation), facing east (-90° azimuth).
 * 
 * @return true if successfully reached safe position
 * @return false if movement failed or limit switches triggered
 */
bool motor_move_to_safe_position(void);

/**
 * @brief Move panel to reference calibration position
 * 
 * Moves to known reference position for system calibration:
 * - Elevation: Local latitude angle (optimal winter sun angle)
 * - Azimuth: True south (180° magnetic + declination correction)
 * 
 * @return true if reference position reached successfully
 * @return false if movement failed (check limit switches)
 */
bool motor_move_to_reference_position(void);

#endif