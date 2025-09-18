#include "solar.h"
#include <math.h>
#include <time.h>
#include "esp_log.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Solar Algorithm Implementation                                        │
    │                                                                       │
    │ Based on NOAA Solar Position Calculator algorithms:                   │
    │ https://gml.noaa.gov/grad/solcalc/solareqns.PDF                      │
    │                                                                       │
    │ Key formulas:                                                         │
    │ - Mean anomaly: M = 357.5291 + 0.98560028 * n                        │
    │ - Equation of center: C = 1.9148*sin(M) + 0.02*sin(2M) + ...         │
    │ - Solar longitude: λ = M + C + 180 + perihelion                       │
    │ - Declination: δ = arcsin(sin(λ) * sin(obliquity))                    │
    │ - Hour angle: H = 15° * (LST - 12)                                    │
    │                                                                       │
    │ Debugging tips:                                                       │
    │ - Compare results with NOAA online calculator for validation          │
    │ - Check timezone handling (all internal calculations use UTC)         │
    │ - Verify GPS coordinates (negative longitude = West)                  │
    │ - Enable DEBUG logging to see intermediate calculation steps          │
    └───────────────────────────────────────────────────────────────────────┘
*/

#define TAG "SOLAR"

// Unit conversion helpers
static double deg2rad(double d){ return d * M_PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / M_PI; }

// Clamp value to range [lo, hi]
static double clamp(double v, double lo, double hi){ 
    return v < lo ? lo : (v > hi ? hi : v); 
}

/*
    Find UTC midnight for the day containing t_utc.
    Also compute day-of-year (1-366) needed for solar declination.
    
    This is used by sunrise/sunset calculations to work within a single UTC day.
*/
static void utc_day_start(time_t t, time_t *day0_out, int *yday_out){
    struct tm *utc = gmtime(&t);
    
    // Compute seconds since UTC midnight
    time_t day0 = t - (utc->tm_hour * 3600 + utc->tm_min * 60 + utc->tm_sec);
    
    if (day0_out) *day0_out = day0;
    if (yday_out) *yday_out = utc->tm_yday + 1; // tm_yday is 0-based, we want 1-366
    
    ESP_LOGV(TAG, "UTC day start: %ld (day %d of year)", (long)day0, utc->tm_yday + 1);
}

double solar_julian_day(const struct tm *utc){
    // Standard Julian Day calculation with Gregorian calendar correction
    int y = utc->tm_year + 1900;
    int m = utc->tm_mon + 1;        // tm_mon is 0-based
    int d = utc->tm_mday;
    
    // For January/February, treat as months 13/14 of previous year
    if (m <= 2){ 
        y--; 
        m += 12; 
    }
    
    // Gregorian calendar correction (valid for dates after 1582-10-15)
    int A = y / 100;
    int B = 2 - A + A / 4;
    
    // Time of day as fraction (0.0 = midnight, 0.5 = noon)
    double dayfrac = (utc->tm_hour + utc->tm_min / 60.0 + utc->tm_sec / 3600.0) / 24.0;
    
    // Standard Julian Day formula
    double JD = floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + d + dayfrac + B - 1524.5;
    
    ESP_LOGV(TAG, "Julian Day: %04d-%02d-%02d %02d:%02d:%02d UTC → %.3f", 
             utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
             utc->tm_hour, utc->tm_min, utc->tm_sec, JD);
    
    return JD;
}

sun_pos_t solar_compute(double lat_deg, double lon_deg, time_t t_utc){
    sun_pos_t s = {0};
    
    // Convert time to Julian Day (astronomical standard reference)
    struct tm *utc = gmtime(&t_utc);
    double JD = solar_julian_day(utc);
    
    // Days since J2000.0 epoch (January 1, 2000, 12:00 UTC)
    double n = JD - 2451545.0;
    ESP_LOGD(TAG, "Days since J2000.0: %.3f", n);
    
    // Mean anomaly (Earth's position in elliptical orbit)
    double M = fmod(357.5291 + 0.98560028 * n, 360.0);
    ESP_LOGV(TAG, "Mean anomaly: %.3f°", M);
    
    // Equation of center (correction for elliptical orbit)
    double C = 1.9148 * sin(deg2rad(M)) + 
               0.02 * sin(deg2rad(2 * M)) + 
               0.0003 * sin(deg2rad(3 * M));
    ESP_LOGV(TAG, "Equation of center: %.4f°", C);
    
    // Ecliptic longitude of sun (apparent position relative to Earth)
    double lambda = fmod(M + C + 180 + 102.9372, 360.0);
    ESP_LOGV(TAG, "Solar longitude: %.3f°", lambda);
    
    // Solar declination (sun's latitude in celestial coordinates)
    // Uses fixed obliquity of 23.44° (accurate enough for tracking)
    double delta = asin(sin(deg2rad(lambda)) * sin(deg2rad(23.44)));
    ESP_LOGV(TAG, "Solar declination: %.3f°", rad2deg(delta));
    
    // Local Solar Time (corrects for longitude offset from GMT)
    double lst = utc->tm_hour + utc->tm_min / 60.0 + utc->tm_sec / 3600.0 + lon_deg / 15.0;
    ESP_LOGV(TAG, "Local Solar Time: %.3f hours", lst);
    
    // Hour angle (sun's position relative to local noon)
    // Positive = afternoon, negative = morning
    double H = deg2rad((lst - 12.0) * 15.0);
    ESP_LOGV(TAG, "Hour angle: %.3f° (%.3f rad)", rad2deg(H), H);
    
    // Convert to observer's local coordinates
    double lat = deg2rad(lat_deg);
    
    // Solar elevation (altitude above horizon)
    double elev = asin(sin(delta) * sin(lat) + cos(delta) * cos(lat) * cos(H));
    s.elevation_deg = rad2deg(elev);
    
    // Solar azimuth (bearing from North)
    // atan2 handles quadrant correctly, then convert to 0-360° from North
    double az = atan2(sin(H), cos(H) * sin(lat) - tan(delta) * cos(lat));
    s.azimuth_deg = fmod(rad2deg(az) + 180.0 + 360.0, 360.0);
    
    // Daylight determination (simple elevation check)
    s.is_daylight = s.elevation_deg > 0.0;
    
    ESP_LOGD(TAG, "Solar position: Az=%.2f° El=%.2f° (daylight=%s) at %.3f,%.3f", 
             s.azimuth_deg, s.elevation_deg, s.is_daylight ? "yes" : "no", lat_deg, lon_deg);
    
    return s;
}

solar_events_t solar_events(double lat_deg, double lon_deg, time_t t_utc){
    solar_events_t ev = {0};
    
    // Get UTC day boundaries and day-of-year
    time_t day0; 
    int yday;
    utc_day_start(t_utc, &day0, &yday);
    
    ESP_LOGD(TAG, "Computing sunrise/sunset for day %d at %.4f,%.4f", yday, lat_deg, lon_deg);
    
    // Solar declination varies throughout the year (seasonal tilt effect)
    double gamma = 2.0 * M_PI / 365.0 * (yday - 1);
    ESP_LOGV(TAG, "Gamma (day angle): %.4f rad", gamma);
    
    // Equation of time: correction for Earth's elliptical orbit and axial tilt
    // This accounts for the "analemma" effect (solar noon varies ±16 minutes)
    double EoT = 229.18 * (0.000075 + 
                          0.001868 * cos(gamma) - 0.032077 * sin(gamma) -
                          0.014615 * cos(2 * gamma) - 0.040849 * sin(2 * gamma));
    ESP_LOGV(TAG, "Equation of time: %.2f minutes", EoT);
    
    // Solar declination for this day of year
    double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma) -
                  0.006758 * cos(2 * gamma) + 0.000907 * sin(2 * gamma) -
                  0.002697 * cos(3 * gamma) + 0.00148 * sin(3 * gamma);
    ESP_LOGV(TAG, "Solar declination: %.4f rad (%.2f°)", decl, rad2deg(decl));
    
    // Standard sunrise/sunset elevation: -0.833° 
    // Accounts for solar disk radius (0.25°) + atmospheric refraction (0.583°)
    double lat = deg2rad(lat_deg);
    double h0 = deg2rad(-0.833);
    
    // Hour angle at sunrise/sunset (when sun crosses h0 elevation)
    double cosH0 = (sin(h0) - sin(lat) * sin(decl)) / (cos(lat) * cos(decl));
    
    // Check for polar day/night conditions
    if (cosH0 > 1.0) {
        // Polar night: sun never rises above -0.833°
        ESP_LOGD(TAG, "Polar night: cosH0=%.3f > 1.0", cosH0);
        ev.has_sunrise = false;
        ev.has_sunset = false;
        return ev;
    }
    
    if (cosH0 < -1.0) {
        // Midnight sun: sun never sets below -0.833°
        ESP_LOGD(TAG, "Midnight sun: cosH0=%.3f < -1.0", cosH0);
        ev.has_sunrise = false;
        ev.has_sunset = false;
        return ev;
    }
    
    // Normal case: compute sunrise and sunset times
    double H0 = acos(clamp(cosH0, -1.0, 1.0));          // Hour angle in radians
    double H0_min = 4.0 * rad2deg(H0);                  // Convert to minutes
    ESP_LOGV(TAG, "Sunrise hour angle: %.4f rad (%.2f°, %.1f min)", H0, rad2deg(H0), H0_min);
    
    // Solar noon time in minutes from UTC midnight
    double noon_min = 720.0 - 4.0 * lon_deg - EoT;     // 720 = 12:00 in minutes
    ESP_LOGV(TAG, "Solar noon: %.1f minutes from UTC midnight", noon_min);
    
    // Sunrise/sunset times relative to solar noon
    double rise_min = noon_min - H0_min;
    double set_min = noon_min + H0_min;
    
    ESP_LOGD(TAG, "Sunrise: %.1f min (%.2f:%.0f), Sunset: %.1f min (%.2f:%.0f)", 
             rise_min, floor(rise_min / 60), fmod(rise_min, 60),
             set_min, floor(set_min / 60), fmod(set_min, 60));
    
    // Convert to epoch seconds (handle potential day boundary crossings)
    int rise_sec = (int)lrint(rise_min * 60.0);
    int set_sec = (int)lrint(set_min * 60.0);
    
    ev.sunrise_utc = day0 + rise_sec;
    ev.sunset_utc = day0 + set_sec;
    ev.has_sunrise = true;
    ev.has_sunset = true;
    
    ESP_LOGD(TAG, "Final times: sunrise=%ld, sunset=%ld", (long)ev.sunrise_utc, (long)ev.sunset_utc);
    
    return ev;
}