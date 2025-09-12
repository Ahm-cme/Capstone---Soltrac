#ifndef SOLAR_CALC_H
#define SOLAR_CALC_H

#include "types.h"
#include <time.h>

/**
 * @brief Calculate precise sun position for GPS-based solar tracking
 * 
 * Uses astronomical algorithms to determine sun's azimuth and elevation
 * angles for any location and time. Essential for accurate solar panel
 * positioning throughout the day and year.
 * 
 * Algorithm based on:
 * - Julian day calculation for astronomical time
 * - Solar declination (seasonal variation)  
 * - Equation of time (Earth's orbital eccentricity)
 * - Hour angle from solar noon
 * - Spherical trigonometry for position conversion
 * 
 * @param loc GPS coordinates (latitude, longitude, altitude)
 * @param now Current UTC time (from GPS or RTC)
 * @return sun_position_t with azimuth (0°=North, 180°=South) and elevation angles
 * 
 * @note Accuracy: ±0.01° typical, suitable for solar tracking
 * @note Time must be UTC - function handles timezone conversion internally
 */
sun_position_t solar_calc_position(gps_location_t loc, time_t now);

/**
 * @brief Check if sun is visible and worth tracking
 * 
 * Determines if sun is above minimum elevation threshold for solar tracking.
 * Below minimum elevation (typically 5°), atmospheric effects and shadows
 * make tracking ineffective.
 * 
 * @param pos Sun position from solar_calc_position()
 * @return true if sun is above MIN_SUN_ELEVATION threshold
 * @return false if sun is below horizon or too low for effective tracking
 * 
 * @note Use this to decide when to activate/deactivate tracking
 * @note Prevents unnecessary motor movement during sunrise/sunset
 */
bool solar_is_visible(sun_position_t pos);

/**
 * @brief Normalize angle to 0-360° range
 * 
 * Utility function to handle angle wraparound for compass headings.
 * Ensures azimuth angles are always in standard 0-360° range.
 * 
 * @param a Input angle in degrees (any value)
 * @return Equivalent angle in 0-360° range
 */
double solar_normalize_360(double a);

#endif