#include "solar_calc.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <time.h>

// Mathematical Constants
#define PI 3.141592653589793
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

// Astronomical Constants
#define JULIAN_EPOCH_2000   2451545.0    // J2000.0 epoch (Jan 1, 2000 12:00 UTC)
#define EARTH_OBLIQUITY     23.439       // Earth's axial tilt in degrees
#define MINUTES_PER_DEGREE  4.0          // Minutes of time per degree longitude

static const char *TAG = "SOLAR";

/**
 * @brief Calculate Julian Day Number for astronomical calculations
 * 
 * Converts Gregorian calendar date/time to Julian Day Number, which is
 * the standard astronomical time system. Required for precise solar
 * position calculations.
 * 
 * @param Y Year (e.g., 2024)
 * @param M Month (1-12)
 * @param D Day of month (1-31)
 * @param h Hour (0-23)
 * @param m Minute (0-59)
 * @param s Second (0-59)
 * @return Julian Day Number (fractional days since Jan 1, 4713 BCE)
 */
static double julian_day(int Y, int M, int D, int h, int m, int s) {
    // Handle January/February as months 13/14 of previous year
    // (Required for Julian calendar algorithm)
    if (M <= 2) {
        Y--;
        M += 12;
    }
    
    // Gregorian calendar correction
    int A = Y / 100;
    int B = 2 - A + A / 4;
    
    // Calculate Julian Day Number
    double JD = (int)(365.25 * (Y + 4716)) + 
                (int)(30.6001 * (M + 1)) + 
                D + B - 1524.5;
    
    // Add fractional day from time
    JD += (h + m/60.0 + s/3600.0) / 24.0;
    
    return JD;
}

/**
 * @brief Calculate Equation of Time correction
 * 
 * The Equation of Time accounts for Earth's elliptical orbit and axial tilt,
 * which causes the sun's apparent motion to vary throughout the year.
 * This correction is essential for accurate solar noon determination.
 * 
 * @param jd Julian Day Number
 * @return Equation of Time in minutes (range: -16 to +14 minutes)
 */
static double eq_time(double jd) {
    // Days since J2000.0 epoch
    double n = jd - JULIAN_EPOCH_2000;
    
    // Mean solar longitude (degrees)
    double L = fmod(280.460 + 0.9856474 * n, 360.0);
    
    // Mean anomaly (angle of Earth in elliptical orbit)
    double g = DEG2RAD * fmod(357.528 + 0.9856003 * n, 360.0);
    
    // True solar longitude (corrected for orbital eccentricity)
    double lambda = DEG2RAD * (L + 1.915 * sin(g) + 0.020 * sin(2*g));
    
    // Right ascension of the sun
    double alpha = atan2(cos(DEG2RAD * EARTH_OBLIQUITY) * sin(lambda), cos(lambda));
    
    // Equation of time in degrees, then convert to minutes
    double E = (L * DEG2RAD - alpha) * RAD2DEG;
    
    // Normalize to ±180° range
    while (E > 180) E -= 360;
    while (E <= -180) E += 360;
    
    return MINUTES_PER_DEGREE * E;  // Convert degrees to minutes
}

/**
 * @brief Calculate solar declination angle
 * 
 * Solar declination is the angle between the sun's rays and the Earth's
 * equatorial plane. It varies seasonally from +23.44° (summer solstice)
 * to -23.44° (winter solstice).
 * 
 * @param jd Julian Day Number
 * @return Solar declination in degrees (-23.44° to +23.44°)
 */
static double decl(double jd) {
    // Days since J2000.0 epoch
    double n = jd - JULIAN_EPOCH_2000;
    
    // Mean solar longitude
    double L = fmod(280.460 + 0.9856474 * n, 360.0);
    
    // Mean anomaly
    double g = DEG2RAD * fmod(357.528 + 0.9856003 * n, 360.0);
    
    // True solar longitude (accounts for orbital eccentricity)
    double lambda = L + 1.915 * sin(g) + 0.020 * sin(2*g);
    
    // Solar declination using Earth's axial tilt
    return asin(sin(DEG2RAD * EARTH_OBLIQUITY) * sin(DEG2RAD * lambda)) * RAD2DEG;
}

double solar_normalize_360(double a) {
    while (a < 0) a += 360;
    while (a >= 360) a -= 360;
    return a;
}

sun_position_t solar_calc_position(gps_location_t loc, time_t now) {
    sun_position_t s = {0};
    
    // Validate input GPS location
    if (fabs(loc.latitude) > 90.0 || fabs(loc.longitude) > 180.0) {
        ESP_LOGE(TAG, "Invalid GPS coordinates: lat=%.6f, lon=%.6f", loc.latitude, loc.longitude);
        return s;  // Return zero position for invalid input
    }
    
    // Convert UTC time to calendar components
    struct tm *utc = gmtime(&now);
    if (!utc) {
        ESP_LOGE(TAG, "Failed to convert timestamp to UTC time");
        return s;
    }
    
    // Calculate Julian Day for astronomical algorithms
    double jd = julian_day(utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
                          utc->tm_hour, utc->tm_min, utc->tm_sec);
    
    // Get solar parameters for this date
    double declination = decl(jd);           // Seasonal solar declination
    double eot = eq_time(jd);                // Equation of time correction
    
    // Calculate solar time (local time corrected for longitude and equation of time)
    double solar_time = utc->tm_hour + utc->tm_min/60.0 + utc->tm_sec/3600.0 +
                       loc.longitude/15.0 +  // Longitude correction (15°/hour)
                       eot/60.0;             // Equation of time correction
    
    // Hour angle: angular displacement from solar noon
    // Solar noon = 0°, morning = negative, afternoon = positive
    double hour_angle = (solar_time - 12.0) * 15.0;  // 15° per hour
    
    // Convert to radians for trigonometric calculations
    double lat_r = loc.latitude * DEG2RAD;
    double dec_r = declination * DEG2RAD;
    double ha_r = hour_angle * DEG2RAD;
    
    // Calculate solar elevation using spherical trigonometry
    // Formula: sin(elevation) = sin(declination)*sin(latitude) + 
    //                           cos(declination)*cos(latitude)*cos(hour_angle)
    double elev_r = asin(sin(dec_r) * sin(lat_r) + 
                        cos(dec_r) * cos(lat_r) * cos(ha_r));
    s.elevation = elev_r * RAD2DEG;
    
    // Calculate solar azimuth (0° = North, 180° = South)
    // Formula accounts for both hour angle and observer's latitude
    double az_r = atan2(sin(ha_r),
                       cos(ha_r) * sin(lat_r) - tan(dec_r) * cos(lat_r));
    s.azimuth = solar_normalize_360(az_r * RAD2DEG + 180.0);
    
    // Log detailed solar calculation for debugging (every hour)
    static int last_hour = -1;
    if (utc->tm_hour != last_hour) {
        ESP_LOGI(TAG, "🌞 Solar Position Calculation:");
        ESP_LOGI(TAG, "  Location: %.6f°, %.6f° (lat, lon)", loc.latitude, loc.longitude);
        ESP_LOGI(TAG, "  UTC Time: %04d-%02d-%02d %02d:%02d:%02d", 
                 utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
                 utc->tm_hour, utc->tm_min, utc->tm_sec);
        ESP_LOGI(TAG, "  Solar Time: %.2f hours", solar_time);
        ESP_LOGI(TAG, "  Declination: %.2f°", declination);
        ESP_LOGI(TAG, "  Hour Angle: %.2f°", hour_angle);
        ESP_LOGI(TAG, "  Result: Azimuth=%.1f°, Elevation=%.1f°", s.azimuth, s.elevation);
        last_hour = utc->tm_hour;
    }
    
    return s;
}

bool solar_is_visible(sun_position_t pos) {
    // Check if sun is above minimum tracking threshold
    bool visible = pos.elevation > MIN_SUN_ELEVATION;
    
    // Log tracking status changes
    static bool was_visible = false;
    if (visible != was_visible) {
        if (visible) {
            ESP_LOGI(TAG, "🌅 Sun tracking ACTIVE - Elevation: %.1f° (above %.1f° threshold)", 
                     pos.elevation, MIN_SUN_ELEVATION);
        } else {
            ESP_LOGI(TAG, "🌇 Sun tracking INACTIVE - Elevation: %.1f° (below %.1f° threshold)", 
                     pos.elevation, MIN_SUN_ELEVATION);
        }
        was_visible = visible;
    }
    
    return visible;
}