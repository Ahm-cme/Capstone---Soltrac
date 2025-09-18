#pragma once
/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ GPS (MAX‑M10S over I2C)                                               │
    │ - Lightweight UBX NAV‑PVT poller and last‑fix cache                  │
    │ - Used by tracking to get lat/lon/time and basic motion info         │
    └───────────────────────────────────────────────────────────────────────┘

    Wiring (project default):
    - I2C bus: I2C_NUM_0, SDA=GPIO18, SCL=GPIO19, 400 kHz
    - GPS address: 0x42 (u‑blox I2C default)

    Notes :
    - We only use UBX NAV‑PVT. No NMEA, no continuous streaming.
    - We actively poll once per loop; keep stack small and code simple.
    - If you later add time sync from GPS to RTC: do it where a valid fix exists.
*/

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"

typedef struct {
    double   latitude;          // degrees
    double   longitude;         // degrees
    double   altitude_m;        // meters (MSL)
    uint8_t  fix_type;          // 0=no fix, 2=2D, 3=3D
    uint8_t  num_satellites;    // SVs used
    bool     valid;             // true if fields are valid
    time_t   timestamp;         // local epoch when fix parsed (not GPS time)
    float    ground_speed_mps;  // m/s
    float    heading_deg;       // degrees (course over ground)
} gps_data_t;

typedef struct {
    int      i2c_port;   // I2C_NUM_0
    int      sda_io;     // GPIO18 (project default)
    int      scl_io;     // GPIO19 (project default)
    uint32_t clk_hz;     // 400000 typical
    uint8_t  addr;       // 0x42 (u‑blox)
} gps_cfg_t;

/*
    Initialize I2C + basic GPS configuration.
    - Sets measurement rate to 1 Hz (safe default).
    - Returns ESP_OK on success.
*/
esp_err_t gps_init(const gps_cfg_t *cfg);

/*
    Poll one NAV‑PVT message and decode it.
    - Returns true if a valid fix is parsed and 'out' is filled.
    - Also updates the internal last‑fix cache on success.
*/
bool gps_poll_nav_pvt(gps_data_t *out);

/*
    Copy the last known good fix into 'out'.
    - Returns false if no valid fix has ever been parsed this boot.
*/
bool gps_get_last(gps_data_t *out);