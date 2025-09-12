#ifndef TYPES_H
#define TYPES_H

#include <stdbool.h>
#include <time.h>

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    bool   valid;
} gps_location_t;

typedef struct {
    double azimuth;    // degrees 0-360
    double elevation;  // degrees
} sun_position_t;

typedef struct {
    int    azimuth_steps;      // Legacy stepper motor steps (not used with actuators)
    int    elevation_steps;    // Legacy stepper motor steps (not used with actuators)
    double azimuth_deg;        // Current azimuth position in degrees
    double elevation_deg;      // Current elevation position in degrees
} panel_position_t;

#endif