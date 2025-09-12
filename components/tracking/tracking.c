#include "tracking.h"
#include "config.h"
#include "gps.h"
#include "solar_calc.h"
#include "motor_control.h"
#include "battery.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <time.h>

static const char *TAG = "TRACK";

// Panel Position Tracking (Updated for Linear Actuators)
static struct {
    float azimuth_deg;      // Current azimuth position (degrees)
    float elevation_deg;    // Current elevation position (degrees)
    float target_az_deg;    // Target azimuth for next move
    float target_el_deg;    // Target elevation for next move
    bool position_known;    // True if position has been calibrated
    uint32_t last_move_time; // Timestamp of last movement
} panel_state = {0};

static gps_location_t current_location;     // Current GPS coordinates
static bool first_run_completed = false;   // Track initialization state

/**
 * @brief Update panel position tracking after movement
 * @param new_az_deg New azimuth position in degrees
 * @param new_el_deg New elevation position in degrees
 */
static void update_panel_position(float new_az_deg, float new_el_deg) {
    panel_state.azimuth_deg = new_az_deg;
    panel_state.elevation_deg = new_el_deg;
    panel_state.last_move_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Panel position updated: Az=%.1f°, El=%.1f°", 
             panel_state.azimuth_deg, panel_state.elevation_deg);
}

/**
 * @brief Move panel to track calculated sun position
 * @param sun Current sun position from solar calculations
 */
static void move_to_sun_position(sun_position_t sun) {
    // Validate sun position is reasonable
    if (sun.azimuth < 0 || sun.azimuth >= 360 || 
        sun.elevation < -90 || sun.elevation > 90) {
        ESP_LOGE(TAG, "Invalid sun position: Az=%.1f°, El=%.1f°", 
                 sun.azimuth, sun.elevation);
        return;
    }
    
    // Apply mechanical limits based on actuator range
    float target_elevation = sun.elevation;
    float target_azimuth = sun.azimuth;
    
    // Clamp elevation to safe mechanical range
    if (target_elevation > MAX_ELEVATION_DEG) {
        ESP_LOGW(TAG, "Sun elevation %.1f° exceeds max %.1f° - clamping", 
                 sun.elevation, MAX_ELEVATION_DEG);
        target_elevation = MAX_ELEVATION_DEG;
    }
    if (target_elevation < MIN_ELEVATION_DEG) {
        ESP_LOGD(TAG, "Sun elevation %.1f° below min %.1f° - clamping", 
                 sun.elevation, MIN_ELEVATION_DEG);
        target_elevation = MIN_ELEVATION_DEG;
    }
    
    // Calculate movement needed from current position
    float az_movement = target_azimuth - panel_state.azimuth_deg;
    float el_movement = target_elevation - panel_state.elevation_deg;
    
    // Handle azimuth wraparound (shortest path around 360°)
    if (az_movement > 180.0f) az_movement -= 360.0f;
    if (az_movement < -180.0f) az_movement += 360.0f;
    
    ESP_LOGI(TAG, "🌞 Sun tracking: Az=%.1f°→%.1f° (%.1f°), El=%.1f°→%.1f° (%.1f°)",
             panel_state.azimuth_deg, target_azimuth, az_movement,
             panel_state.elevation_deg, target_elevation, el_movement);
    
    // Check if movement is significant enough to warrant motor operation
    bool az_needs_move = fabs(az_movement) > MIN_MOVEMENT_THRESHOLD_DEG;
    bool el_needs_move = fabs(el_movement) > MIN_MOVEMENT_THRESHOLD_DEG;
    
    if (!az_needs_move && !el_needs_move) {
        ESP_LOGD(TAG, "Movement below threshold (%.1f°) - staying put", 
                 MIN_MOVEMENT_THRESHOLD_DEG);
        return;
    }
    
    // Store target positions for reference
    panel_state.target_az_deg = target_azimuth;
    panel_state.target_el_deg = target_elevation;
    
    // Perform movement using updated motor control functions
    if (az_needs_move && el_needs_move) {
        // Both axes need movement - use simultaneous function
        ESP_LOGI(TAG, "Moving both axes simultaneously");
        motor_move_to_sun_position(target_azimuth, target_elevation, MOTOR_SPEED_PERCENT);
        update_panel_position(target_azimuth, target_elevation);
    } else {
        // Single axis movement
        if (az_needs_move) {
            ESP_LOGI(TAG, "Moving azimuth axis only");
            motor_move_azimuth_degrees(target_azimuth, MOTOR_SPEED_PERCENT);
            update_panel_position(target_azimuth, panel_state.elevation_deg);
        }
        if (el_needs_move) {
            ESP_LOGI(TAG, "Moving elevation axis only");
            motor_move_elevation_degrees(target_elevation, MOTOR_SPEED_PERCENT);
            update_panel_position(panel_state.azimuth_deg, target_elevation);
        }
    }
    
    // Calculate final tracking accuracy
    float az_error = fabs(target_azimuth - panel_state.azimuth_deg);
    float el_error = fabs(target_elevation - panel_state.elevation_deg);
    
    if (az_error <= TRACKING_TOLERANCE_DEG && el_error <= TRACKING_TOLERANCE_DEG) {
        ESP_LOGI(TAG, "✅ Sun tracking accurate: Az error=%.2f°, El error=%.2f°", 
                 az_error, el_error);
    } else {
        ESP_LOGW(TAG, "⚠️  Tracking error: Az=%.2f°, El=%.2f° (tolerance=%.1f°)", 
                 az_error, el_error, TRACKING_TOLERANCE_DEG);
    }
}

/**
 * @brief Move panel to safe storage position for sleep/shutdown
 */
static void move_to_safe_position(void) {
    ESP_LOGI(TAG, "🏠 Moving to safe storage position for shutdown");
    
    // Use motor control safe position function
    if (motor_move_to_safe_position()) {
        // Update tracking to reflect safe position
        update_panel_position(-90.0f, 0.0f);  // East-facing, horizontal
        ESP_LOGI(TAG, "✅ Safe position reached");
    } else {
        ESP_LOGE(TAG, "❌ Failed to reach safe position");
    }
}

void tracking_init(void) {
    ESP_LOGI(TAG, "Initializing GPS-based solar tracking system");
    
    // Get initial GPS location (GPS or fallback coordinates)
    current_location = gps_get_active_or_fallback();
    ESP_LOGI(TAG, "Using location: %.6f°, %.6f° (alt: %.1fm)", 
             current_location.latitude, current_location.longitude, current_location.altitude);
    
    // Initialize panel state
    panel_state.azimuth_deg = 0.0f;
    panel_state.elevation_deg = 0.0f;
    panel_state.position_known = false;
    panel_state.last_move_time = 0;
    first_run_completed = false;
    
    ESP_LOGI(TAG, "Solar tracking system ready");
    ESP_LOGW(TAG, "⚠️  Panel position unknown - call tracking_first_run() to calibrate");
}

void tracking_first_run(void) {
    if (first_run_completed) {
        ESP_LOGD(TAG, "First run already completed");
        return;
    }
    
    ESP_LOGI(TAG, "🚀 Starting first-run solar tracking sequence");
    
    // Try to get fresh GPS coordinates
    gps_location_t fresh_gps;
    if (gps_read_location(&fresh_gps) && fresh_gps.valid) {
        current_location = fresh_gps;
        ESP_LOGI(TAG, "Updated GPS location: %.6f°, %.6f°", 
                 current_location.latitude, current_location.longitude);
    } else {
        ESP_LOGW(TAG, "Using fallback GPS coordinates for initial tracking");
    }
    
    // Move to reference position for calibration
    ESP_LOGI(TAG, "Moving to reference calibration position...");
    if (motor_move_to_reference_position()) {
        // Reference position sets known coordinates
        panel_state.azimuth_deg = REFERENCE_AZIMUTH_DEG;    // True south
        panel_state.elevation_deg = REFERENCE_ELEVATION_DEG; // Local latitude
        panel_state.position_known = true;
        
        ESP_LOGI(TAG, "✅ Reference position calibration complete");
        ESP_LOGI(TAG, "Panel calibrated at: Az=%.1f° (south), El=%.1f° (latitude)",
                 panel_state.azimuth_deg, panel_state.elevation_deg);
    } else {
        ESP_LOGE(TAG, "❌ Reference position calibration failed");
        return;
    }
    
    // Calculate current sun position
    time_t now = time(NULL);
    sun_position_t sun = solar_calc_position(current_location, now);
    
    ESP_LOGI(TAG, "Current sun position: Az=%.1f°, El=%.1f°", 
             sun.azimuth, sun.elevation);
    
    // Move to sun if visible
    if (solar_is_visible(sun)) {
        ESP_LOGI(TAG, "Sun is visible - beginning tracking");
        move_to_sun_position(sun);
    } else {
        ESP_LOGI(TAG, "Sun not visible - staying at reference position");
    }
    
    first_run_completed = true;
    ESP_LOGI(TAG, "🌞 First-run sequence complete - system ready for continuous tracking");
}

void tracking_loop(void) {
    if (!first_run_completed) {
        ESP_LOGW(TAG, "First run not completed - call tracking_first_run() first");
        return;
    }
    
    ESP_LOGI(TAG, "🔄 Starting continuous solar tracking loop");
    ESP_LOGI(TAG, "Update interval: %d seconds, GPS refresh: every 10 cycles", 
             UPDATE_INTERVAL_SEC);
    
    int gps_update_counter = 0;
    int tracking_cycle = 0;
    
    while (true) {
        tracking_cycle++;
        ESP_LOGI(TAG, "--- Tracking Cycle #%d ---", tracking_cycle);
        
        // Critical battery check - exit if power too low
        if (!battery_is_ok()) {
            float voltage = battery_read_voltage();
            ESP_LOGW(TAG, "🔋 Battery voltage low (%.2fV) - ending tracking for power conservation", voltage);
            break;
        }
        
        // Optional: Stop tracking when battery full (solar charging complete)
        if (battery_is_full()) {
            ESP_LOGI(TAG, "🔋 Battery fully charged - tracking complete for today");
            break;
        }
        
        // Periodic GPS location update (every 10 cycles to save power)
        gps_update_counter++;
        if (gps_update_counter >= 10) {
            gps_location_t new_location;
            if (gps_read_location(&new_location) && new_location.valid) {
                current_location = new_location;
                ESP_LOGI(TAG, "📍 GPS updated: %.6f°, %.6f°", 
                         current_location.latitude, current_location.longitude);
            } else {
                ESP_LOGD(TAG, "GPS update failed - using cached coordinates");
            }
            gps_update_counter = 0;
        }
        
        // Calculate current sun position
        time_t current_time = time(NULL);
        sun_position_t sun = solar_calc_position(current_location, current_time);
        
        // Check if sun is worth tracking
        if (!solar_is_visible(sun)) {
            ESP_LOGI(TAG, "🌇 Sun below tracking threshold (%.1f° < %.1f°) - ending tracking", 
                     sun.elevation, MIN_SUN_ELEVATION);
            break;
        }
        
        // Perform sun tracking movement
        move_to_sun_position(sun);
        
        // Log tracking status for monitoring
        ESP_LOGI(TAG, "💤 Tracking cycle complete - sleeping %d seconds until next update", 
                 UPDATE_INTERVAL_SEC);
        
        // Wait for next tracking interval
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_SEC * 1000));
    }
    
    ESP_LOGI(TAG, "🌙 Solar tracking loop ended after %d cycles", tracking_cycle);
}

void tracking_prepare_sleep(void) {
    ESP_LOGI(TAG, "🌙 Preparing system for sleep/shutdown");
    
    // Move panel to safe storage position
    move_to_safe_position();
    
    // Ensure all motors are stopped
    motor_stop_all();
    
    // Log sleep duration
    uint64_t sleep_us = (uint64_t)SLEEP_INTERVAL_SEC * 1000000ULL;
    ESP_LOGI(TAG, "Configuring deep sleep for %d seconds (%.1f hours)", 
             SLEEP_INTERVAL_SEC, SLEEP_INTERVAL_SEC / 3600.0f);
    
    // Configure wake-up timer for next sunrise
    esp_sleep_enable_timer_wakeup(sleep_us);
    
    ESP_LOGI(TAG, "💤 Entering deep sleep - good night!");
    
    // Enter deep sleep mode
    esp_deep_sleep_start();
}