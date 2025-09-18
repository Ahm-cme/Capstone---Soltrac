#pragma once
#include <stdbool.h>
#include <time.h>

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Solar Position & Event Calculator                                    │
    │ - Computes sun azimuth/elevation for any location and time           │
    │ - Handles sunrise/sunset calculations with NOAA approximations       │
    │ - Core algorithm for solar tracking system                           │
    └───────────────────────────────────────────────────────────────────────┘

    Algorithm notes:
    - Uses simplified VSOP87/NOAA algorithms (accurate to ~0.01° for tracking)
    - All angles in degrees, times in UTC epoch seconds
    - Azimuth: 0°=North, 90°=East, 180°=South, 270°=West
    - Elevation: 0°=horizon, 90°=zenith, negative=below horizon
    - Atmospheric refraction correction: -0.833° for sunrise/sunset

    Coordinate system:
    - Earth-fixed coordinates (not mount coordinates)
    - Tracking system applies mount offsets to convert to panel coordinates
    - Longitude: positive=East, negative=West (standard GPS convention)
    - Latitude: positive=North, negative=South

    Accuracy expectations:
    - Position accuracy: ~±0.01° (better than tracking resolution)
    - Time accuracy: ~±1 minute for sunrise/sunset
    - Valid for years 1950-2050 (beyond that, use full VSOP87)
    - Polar regions: handles midnight sun and polar night correctly

    Performance:
    - Fast enough to compute every 5-15 minutes without issue
    - No trigonometric lookup tables needed (modern MCUs handle this fine)
    - Stack usage: ~100 bytes per call
*/

typedef struct {
    double azimuth_deg;     // 0-360° from North (clockwise when viewed from above)
    double elevation_deg;   // ±90° from horizon (positive = above horizon)
    bool is_daylight;       // true if elevation > 0° (sun is visible)
} sun_pos_t;

/*
    Compute sun position for given location and time.
    
    Uses simplified orbital mechanics with these approximations:
    - Earth orbit treated as elliptical (ignores minor planetary perturbations)
    - Atmospheric refraction ignored for position (only used for rise/set times)
    - Nutation/precession approximated with fixed obliquity (23.44°)
    
    Parameters:
    - lat_deg: latitude in decimal degrees (-90 to +90)
    - lon_deg: longitude in decimal degrees (-180 to +180)  
    - t_utc: UTC time as Unix epoch seconds
    
    Returns: sun position in Earth-fixed coordinates
*/
sun_pos_t solar_compute(double lat_deg, double lon_deg, time_t t_utc);

/*
    Convert UTC time to Julian Day Number (astronomical standard).
    
    Uses standard Julian calendar conversion with leap year corrections.
    Handles the Gregorian calendar reform (post-1582) automatically.
    
    Parameters:
    - utc: broken-down UTC time structure
    
    Returns: Julian Day as fractional double (J2000.0 = 2451545.0)
*/
double solar_julian_day(const struct tm *utc);

typedef struct {
    time_t sunrise_utc;     // UTC epoch seconds for sunrise
    time_t sunset_utc;      // UTC epoch seconds for sunset  
    bool has_sunrise;       // false during polar night
    bool has_sunset;        // false during midnight sun
} solar_events_t;

/*
    Compute sunrise/sunset times using NOAA approximations.
    
    Uses the standard -0.833° elevation for geometric sunrise/sunset:
    - Accounts for solar disk radius (0.25°) 
    - Includes average atmospheric refraction (0.583°)
    - Does NOT account for local terrain (mountains, buildings)
    
    Algorithm steps:
    1. Compute solar declination for the day of year
    2. Calculate equation of time correction
    3. Find hour angles where sun crosses -0.833° elevation
    4. Convert to UTC times accounting for longitude
    
    Polar region handling:
    - Returns has_sunrise=false, has_sunset=false during polar night
    - Returns has_sunrise=false, has_sunset=false during midnight sun
    - Check these flags before using the timestamp values
    
    Parameters:
    - lat_deg: observer latitude (-90 to +90 degrees)
    - lon_deg: observer longitude (-180 to +180 degrees)
    - t_utc: any time during the desired UTC day
    
    Returns: sunrise/sunset events for that UTC calendar day
*/
solar_events_t solar_events(double lat_deg, double lon_deg, time_t t_utc);