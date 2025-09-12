#include "gps.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Configuration for SparkFun GPS-RTK Dead Reckoning Breakout
#define I2C_SCL_PIN       GPIO_NUM_22    // Standard ESP32 I2C SCL (clock)
#define I2C_SDA_PIN       GPIO_NUM_21    // Standard ESP32 I2C SDA (data)  
#define GPS_I2C_ADDRESS   0x42           // Default u-blox GPS module address
#define I2C_MASTER_NUM    I2C_NUM_1      // Use I2C port 1 (port 0 often used by system)
#define I2C_MASTER_FREQ_HZ 400000        // 400kHz fast mode for GPS data throughput

// GPS Module Timing Constants
#define GPS_COLD_START_TIMEOUT_SEC  60   // Maximum time to wait for cold start fix
#define GPS_WARM_START_TIMEOUT_SEC  10   // Maximum time to wait for warm start fix  
#define GPS_RETRY_INTERVAL_MS       1000 // Time between GPS read attempts
#define GPS_MIN_SATELLITES          4    // Minimum satellites for reliable fix
#define GPS_MAX_HDOP               5.0   // Maximum horizontal dilution of precision

static const char *TAG = "GPS";

// GPS State Tracking
static int gps_read_attempts = 0;            // Count of read attempts since init
static uint32_t last_successful_fix_time = 0; // Timestamp of last valid GPS fix
static bool gps_module_responsive = false;   // True if module responding to I2C
static gps_location_t last_valid_fix = {     // Most recent valid GPS data
    .latitude = FALLBACK_LATITUDE,
    .longitude = FALLBACK_LONGITUDE, 
    .altitude = FALLBACK_ALTITUDE,
    .valid = false
};

void gps_init(void) {
    ESP_LOGI(TAG, "Initializing SparkFun GPS-RTK Dead Reckoning (MAX-M10S)");
    
    // Configure I2C master interface
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-ups
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  // (eliminates external resistors)
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    // Apply I2C configuration
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return;
    }
    
    // Install I2C driver with minimal buffer sizes (GPS uses polling, not buffering)
    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "I2C initialized - SCL: GPIO%d, SDA: GPIO%d, Freq: %d Hz", 
             I2C_SCL_PIN, I2C_SDA_PIN, I2C_MASTER_FREQ_HZ);
    
    // Initialize state variables
    gps_read_attempts = 0;
    last_successful_fix_time = 0;
    gps_module_responsive = false;
    
    // Test I2C communication with GPS module
    uint8_t test_data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (GPS_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &test_data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPS module detected at I2C address 0x%02X", GPS_I2C_ADDRESS);
        gps_module_responsive = true;
    } else {
        ESP_LOGW(TAG, "GPS module not responding at I2C address 0x%02X: %s", 
                 GPS_I2C_ADDRESS, esp_err_to_name(ret));
        ESP_LOGW(TAG, "Check wiring: SDA->GPIO21, SCL->GPIO22, VCC->3.3V, GND->GND");
    }
    
    ESP_LOGI(TAG, "GPS initialization complete");
    ESP_LOGI(TAG, "Fallback coordinates: %.6f, %.6f (altitude: %.1fm)", 
             FALLBACK_LATITUDE, FALLBACK_LONGITUDE, FALLBACK_ALTITUDE);
}

bool gps_read_location(gps_location_t *out) {
    if (!out) {
        ESP_LOGE(TAG, "NULL output pointer provided");
        return false;
    }
    
    gps_read_attempts++;
    
    // Check if GPS module is responsive via I2C
    if (!gps_module_responsive) {
        ESP_LOGD(TAG, "GPS module not responsive - using fallback coordinates");
        out->valid = false;
        return false;
    }
    
    /*
     * DEVELOPER NOTE: This is currently a simulation for testing.
     * 
     * In production, this function should:
     * 1. Read NMEA sentences from GPS via I2C
     * 2. Parse GGA/RMC sentences for position data
     * 3. Validate fix quality (satellites, HDOP, etc.)
     * 4. Update last_valid_fix if data is good
     * 
     * SparkFun MAX-M10S provides:
     * - UBX binary protocol (more efficient than NMEA)
     * - Dead reckoning when GPS signals lost
     * - High update rates (up to 25Hz)
     */
    
    if (gps_read_attempts < 3) {
        // Simulate GPS acquisition time (cold start scenario)
        ESP_LOGD(TAG, "GPS acquiring satellites... (attempt %d/3)", gps_read_attempts);
        out->valid = false;
        return false;
    }
    
    // Simulate successful GPS fix after initial attempts
    ESP_LOGI(TAG, "GPS fix acquired! Using fallback coordinates for simulation");
    
    // Update last known good position
    last_valid_fix.latitude = FALLBACK_LATITUDE;
    last_valid_fix.longitude = FALLBACK_LONGITUDE; 
    last_valid_fix.altitude = FALLBACK_ALTITUDE;
    last_valid_fix.valid = true;
    last_successful_fix_time = xTaskGetTickCount();
    
    // Return current position
    *out = last_valid_fix;
    return true;
}

gps_location_t gps_get_active_or_fallback(void) {
    gps_location_t current_pos;
    
    // Try to get fresh GPS data first
    if (gps_read_location(&current_pos) && current_pos.valid) {
        ESP_LOGD(TAG, "Returning fresh GPS coordinates");
        return current_pos;
    }
    
    // No fresh GPS data - check if we have previous valid fix
    if (last_valid_fix.valid) {
        uint32_t time_since_fix = xTaskGetTickCount() - last_successful_fix_time;
        uint32_t age_seconds = time_since_fix * portTICK_PERIOD_MS / 1000;
        
        if (age_seconds < 3600) { // Less than 1 hour old
            ESP_LOGD(TAG, "Using cached GPS fix (%lu seconds old)", age_seconds);
            return last_valid_fix;
        } else {
            ESP_LOGW(TAG, "Cached GPS fix too old (%lu seconds) - using fallback", age_seconds);
        }
    }
    
    // No valid GPS data available - use fallback coordinates
    ESP_LOGW(TAG, "No GPS fix available - using fallback coordinates");
    ESP_LOGW(TAG, "Verify fallback coordinates are correct for your installation site");
    
    gps_location_t fallback = {
        .latitude = FALLBACK_LATITUDE,
        .longitude = FALLBACK_LONGITUDE,
        .altitude = FALLBACK_ALTITUDE,
        .valid = true  // Fallback coordinates are considered "valid" for tracking
    };
    
    return fallback;
}