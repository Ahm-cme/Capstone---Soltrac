#ifndef GPS_H
#define GPS_H

#include "types.h"
#include <stdbool.h>

/**
 * @brief Initialize GPS module and I2C communication
 * 
 * Sets up I2C master interface to communicate with SparkFun GPS-RTK 
 * Dead Reckoning Breakout (MAX-M10S) via Qwiic connector.
 * 
 * I2C Configuration:
 * - Frequency: 400kHz (fast mode for GPS data throughput)
 * - Pull-ups: Enabled (required for reliable I2C communication)
 * - Address: 0x42 (default for u-blox GPS modules)
 * 
 * @note Must be called before any other GPS functions
 * @note Enables internal pull-ups - external pull-ups not required
 */
void gps_init(void);

/**
 * @brief Attempt to read current GPS location
 * 
 * Tries to get a fresh GPS fix from the module. On SparkFun MAX-M10S:
 * - Cold start: 26 seconds typical acquisition time
 * - Warm start: 2 seconds typical
 * - Hot start: <1 second
 * 
 * Dead reckoning provides position estimates when GPS signals lost.
 * 
 * @param out Pointer to structure to store GPS data
 * @return true if fresh GPS data available and valid
 * @return false if no fix available (use fallback coordinates)
 * 
 * @note Sets out->valid = false if no current fix available
 * @note Does NOT block - returns immediately with current status
 */
bool gps_read_location(gps_location_t *out);

/**
 * @brief Get best available location (GPS or fallback)
 * 
 * Returns the most recent valid GPS location, or fallback coordinates
 * if GPS has never achieved a fix. Useful for solar tracking when
 * GPS temporarily unavailable (cloudy conditions, etc).
 * 
 * Fallback coordinates should be set to your installation site:
 * - Accuracy within ~1km acceptable for solar tracking
 * - Coordinates from Google Maps, GPS phone app, etc.
 * 
 * @return gps_location_t structure with best available position
 * @note Always returns valid coordinates (never fails)
 * @note Check .valid field to determine if GPS or fallback data
 */
gps_location_t gps_get_active_or_fallback(void);

#endif