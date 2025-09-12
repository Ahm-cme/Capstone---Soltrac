#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// LOCATION & GPS CONFIGURATION
// ============================================================================

// Fallback coordinates (Auburn, AL area) - UPDATE FOR YOUR LOCATION
#define FALLBACK_LATITUDE   32.6098
#define FALLBACK_LONGITUDE -85.4808
#define FALLBACK_ALTITUDE   200.0

// GPS Hardware Configuration (I2C via Qwiic)
// Note: GPS module uses I2C, pins defined in gps.c
// Standard ESP32 I2C: SDA=GPIO21, SCL=GPIO22

// ============================================================================
// LINEAR ACTUATOR HARDWARE CONFIGURATION
// ============================================================================

// Linear Actuator PWM Control (Cytron MDD20A Driver)
#define ACTUATOR_ELEVATION_PWM_PIN      25    // GPIO25 - Elevation actuator PWM
#define ACTUATOR_ELEVATION_DIR_PIN      26    // GPIO26 - Elevation actuator direction
#define ACTUATOR_AZIMUTH_PWM_PIN        27    // GPIO27 - Azimuth actuator PWM
#define ACTUATOR_AZIMUTH_DIR_PIN        14    // GPIO14 - Azimuth actuator direction

// Actuator Specifications (12V Linear Actuators)
#define ACTUATOR_STROKE_MM              200     // Total stroke length (7.87")
#define ACTUATOR_SAFETY_MARGIN_MM       5       // Safety margin from limits
#define ACTUATOR_SPEED_MM_PER_SEC       11.938  // 0.47 inches/sec converted
#define ACTUATOR_MAX_FORCE_LBS          220     // Force rating

// ============================================================================
// MECHANICAL CALIBRATION (CRITICAL - MUST MEASURE FOR YOUR SETUP)
// ============================================================================

// Conversion factors - depends on your mounting geometry
#define MM_PER_DEGREE_ELEVATION         2.5     // Linear mm per elevation degree
#define MM_PER_DEGREE_AZIMUTH          1.8     // Linear mm per azimuth degree

// Reference Position (for calibration)
#define REFERENCE_ELEVATION_DEG         32.6    // Your local latitude (Auburn, AL)
#define REFERENCE_AZIMUTH_DEG          180.0    // True south
#define REFERENCE_ACTUATOR_EL_MM       100      // Elevation actuator at reference
#define REFERENCE_ACTUATOR_AZ_MM       100      // Azimuth actuator at reference

// Mechanical Travel Limits
#define MAX_ELEVATION_DEG               85.0f   // Maximum panel tilt
#define MIN_ELEVATION_DEG               -5.0f   // Minimum tilt (drainage)
#define MAX_AZIMUTH_DEG                270.0f   // Maximum azimuth travel
#define MIN_AZIMUTH_DEG                 90.0f   // Minimum azimuth travel

// ============================================================================
// SOLAR TRACKING PARAMETERS
// ============================================================================

#define UPDATE_INTERVAL_SEC             300     // 5 minutes between updates
#define MIN_SUN_ELEVATION               5.0f    // Minimum elevation for tracking
#define MIN_MOVEMENT_THRESHOLD_DEG      2.0f    // Minimum movement threshold
#define TRACKING_TOLERANCE_DEG          1.0f    // Acceptable positioning error
#define MOTOR_SPEED_PERCENT             40      // Safe tracking speed

// ============================================================================
// BATTERY MONITORING
// ============================================================================

#define BATTERY_ADC_PIN         6               // ADC1_CH6 (GPIO34) for voltage sensing
#define BATTERY_LOW_VOLTAGE     11.0f           // Shutdown threshold
#define BATTERY_FULL_VOLTAGE    12.6f           // Full charge threshold

// ============================================================================
// USER INTERFACE
// ============================================================================

#define START_BUTTON_PIN        0               // GPIO0 (BOOT button)

// ============================================================================
// POWER MANAGEMENT
// ============================================================================

#define SLEEP_INTERVAL_SEC              3600    // 1 hour sleep
#define SLEEP_HOURS_DEFAULT             8       // Default sleep duration

#endif