#include "motor_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

// PWM Configuration for Cytron MDD20A Motor Driver
#define LEDC_TIMER_USED     LEDC_TIMER_0
#define LEDC_MODE_USED      LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ        1000                  // 1kHz optimal for motor drivers
#define LEDC_RESOLUTION     LEDC_TIMER_10_BIT     // 10-bit = 0-1023 range
#define MAX_DUTY_CYCLE      1023                  // Maximum PWM duty cycle

// Motor Control Parameters
#define MIN_SPEED_PERCENT   10                    // Minimum speed to overcome friction
#define MAX_SPEED_PERCENT   80                    // Maximum safe speed (prevent overheating)
#define RAMP_UP_STEPS       5                     // PWM steps for soft start
#define RAMP_DOWN_STEPS     5                     // PWM steps for soft stop
#define MOVEMENT_TIMEOUT_MS 30000                 // 30 second timeout for movements

// Position Tracking (CRITICAL - No position feedback from actuators!)
static float current_azimuth_degrees = 0.0f;     // Current estimated azimuth position
static float current_elevation_degrees = 0.0f;   // Current estimated elevation position
static bool position_calibrated = false;         // True if reference position set

static const char *TAG = "MOTOR";

/**
 * @brief Convert percentage speed to PWM duty cycle
 * @param speed_percent Speed as percentage (0-100)
 * @return PWM duty cycle value (0-1023)
 */
static int speed_to_duty(int speed_percent) {
    // Clamp speed to safe range
    if (speed_percent < MIN_SPEED_PERCENT) speed_percent = MIN_SPEED_PERCENT;
    if (speed_percent > MAX_SPEED_PERCENT) speed_percent = MAX_SPEED_PERCENT;
    
    return (speed_percent * MAX_DUTY_CYCLE) / 100;
}

/**
 * @brief Calculate movement time based on angle and actuator specifications
 * @param degrees Angle to move (absolute value)
 * @return Movement time in milliseconds
 */
static uint32_t calculate_movement_time_ms(float degrees) {
    // Convert degrees to linear actuator movement
    // This depends on your mechanical setup - MUST BE CALIBRATED
    float mm_per_degree_avg = (MM_PER_DEGREE_ELEVATION + MM_PER_DEGREE_AZIMUTH) / 2.0f;
    float mm_to_move = fabs(degrees) * mm_per_degree_avg;
    
    // Calculate time based on actuator speed (11.938 mm/sec at full speed)
    float movement_time_sec = mm_to_move / ACTUATOR_SPEED_MM_PER_SEC;
    
    // Add safety margin for acceleration/deceleration
    return (uint32_t)(movement_time_sec * 1000.0f * 1.2f);  // 20% margin
}

/**
 * @brief Perform smooth PWM ramp up/down for motor control
 */
static void ramp_pwm(ledc_channel_t channel, int start_duty, int end_duty, int steps) {
    if (steps <= 0) return;
    
    int duty_step = (end_duty - start_duty) / steps;
    
    for (int i = 0; i <= steps; i++) {
        int current_duty = start_duty + (duty_step * i);
        ledc_set_duty(LEDC_MODE_USED, channel, current_duty);
        ledc_update_duty(LEDC_MODE_USED, channel);
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms per step
    }
}

/**
 * @brief Move single axis with direction control and ramping
 */
static bool move_axis_degrees(ledc_channel_t pwm_channel, gpio_num_t dir_pin,
                             float current_pos, float target_pos, int speed_percent,
                             const char* axis_name) {
    
    float degrees_to_move = target_pos - current_pos;
    
    if (fabs(degrees_to_move) < 0.1f) {
        ESP_LOGD(TAG, "%s axis already at target position (%.1f°)", axis_name, target_pos);
        return true;
    }
    
    // Set direction based on movement needed
    bool direction = degrees_to_move > 0;
    gpio_set_level(dir_pin, direction ? 1 : 0);
    
    ESP_LOGI(TAG, "Moving %s: %.1f° → %.1f° (%.1f° %s)", 
             axis_name, current_pos, target_pos, fabs(degrees_to_move),
             direction ? "positive" : "negative");
    
    // Calculate movement parameters
    int target_duty = speed_to_duty(speed_percent);
    uint32_t movement_time = calculate_movement_time_ms(degrees_to_move);
    
    ESP_LOGD(TAG, "%s movement: PWM=%d, Time=%lums", axis_name, target_duty, movement_time);
    
    // Ramp up PWM for smooth start
    ramp_pwm(pwm_channel, 0, target_duty, RAMP_UP_STEPS);
    
    // Main movement phase
    uint32_t main_movement_time = movement_time - ((RAMP_UP_STEPS + RAMP_DOWN_STEPS) * 20);
    if (main_movement_time > 0) {
        vTaskDelay(pdMS_TO_TICKS(main_movement_time));
    }
    
    // Ramp down PWM for smooth stop  
    ramp_pwm(pwm_channel, target_duty, 0, RAMP_DOWN_STEPS);
    
    // Ensure motor is fully stopped
    ledc_set_duty(LEDC_MODE_USED, pwm_channel, 0);
    ledc_update_duty(LEDC_MODE_USED, pwm_channel);
    gpio_set_level(dir_pin, 0);
    
    ESP_LOGI(TAG, "%s movement complete: %.1f°", axis_name, target_pos);
    return true;
}

void motor_init(void) {
    ESP_LOGI(TAG, "Initializing linear actuator motor control");
    ESP_LOGI(TAG, "Hardware: 2x 12V actuators + Cytron MDD20A driver");
    
    // Configure PWM timer for both motor channels
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE_USED,
        .timer_num = LEDC_TIMER_USED,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure elevation actuator PWM channel
    ledc_channel_config_t elevation_pwm = {
        .speed_mode = LEDC_MODE_USED,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_USED,
        .gpio_num = ACTUATOR_ELEVATION_PWM_PIN,  // Uses raw number from config.h
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&elevation_pwm);
    
    // Configure azimuth actuator PWM channel
    ledc_channel_config_t azimuth_pwm = {
        .speed_mode = LEDC_MODE_USED,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_USED,
        .gpio_num = ACTUATOR_AZIMUTH_PWM_PIN,    // Uses raw number from config.h
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&azimuth_pwm);
    
    // Configure direction control pins
    gpio_config_t dir_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ACTUATOR_ELEVATION_DIR_PIN) | 
                       (1ULL << ACTUATOR_AZIMUTH_DIR_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_config);
    
    // Initialize all outputs to safe state
    motor_stop_all();
    
    // Initialize position tracking
    current_azimuth_degrees = 0.0f;
    current_elevation_degrees = 0.0f;
    position_calibrated = false;
    
    ESP_LOGI(TAG, "Motor control initialized successfully");
    ESP_LOGI(TAG, "Elevation: PWM=GPIO%d, DIR=GPIO%d", 
             ACTUATOR_ELEVATION_PWM_PIN, ACTUATOR_ELEVATION_DIR_PIN);
    ESP_LOGI(TAG, "Azimuth: PWM=GPIO%d, DIR=GPIO%d", 
             ACTUATOR_AZIMUTH_PWM_PIN, ACTUATOR_AZIMUTH_DIR_PIN);
    ESP_LOGW(TAG, "⚠️  Position tracking requires calibration before sun tracking");
}

void motor_move_azimuth_degrees(float target_degrees, int speed_percent) {
    ESP_LOGI(TAG, "Moving azimuth to %.1f° at %d%% speed", target_degrees, speed_percent);
    
    if (move_axis_degrees(LEDC_CHANNEL_1, ACTUATOR_AZIMUTH_DIR_PIN,
                         current_azimuth_degrees, target_degrees, speed_percent, "Azimuth")) {
        current_azimuth_degrees = target_degrees;  // Update position tracking
    }
}

void motor_move_elevation_degrees(float target_degrees, int speed_percent) {
    ESP_LOGI(TAG, "Moving elevation to %.1f° at %d%% speed", target_degrees, speed_percent);
    
    if (move_axis_degrees(LEDC_CHANNEL_0, ACTUATOR_ELEVATION_DIR_PIN,
                         current_elevation_degrees, target_degrees, speed_percent, "Elevation")) {
        current_elevation_degrees = target_degrees;  // Update position tracking
    }
}

void motor_move_to_sun_position(float azimuth_degrees, float elevation_degrees, int speed_percent) {
    ESP_LOGI(TAG, "🌞 Moving to sun position: Az=%.1f°, El=%.1f° at %d%% speed", 
             azimuth_degrees, elevation_degrees, speed_percent);
    
    if (!position_calibrated) {
        ESP_LOGW(TAG, "⚠️  System not calibrated - movements may be inaccurate");
        ESP_LOGW(TAG, "Call motor_move_to_reference_position() first");
    }
    
    // Calculate movement times for both axes
    uint32_t az_time = calculate_movement_time_ms(azimuth_degrees - current_azimuth_degrees);
    uint32_t el_time = calculate_movement_time_ms(elevation_degrees - current_elevation_degrees);
    
    ESP_LOGI(TAG, "Estimated movement time: Az=%lums, El=%lums", az_time, el_time);
    
    // For now, move axes sequentially (could be parallelized with FreeRTOS tasks)
    motor_move_azimuth_degrees(azimuth_degrees, speed_percent);
    motor_move_elevation_degrees(elevation_degrees, speed_percent);
    
    ESP_LOGI(TAG, "✅ Sun position reached: Az=%.1f°, El=%.1f°", 
             current_azimuth_degrees, current_elevation_degrees);
}

void motor_stop_all(void) {
    ESP_LOGW(TAG, "🛑 EMERGENCY STOP - All motors halted");
    
    // Stop PWM on both channels
    ledc_set_duty(LEDC_MODE_USED, LEDC_CHANNEL_0, 0);  // Elevation
    ledc_set_duty(LEDC_MODE_USED, LEDC_CHANNEL_1, 0);  // Azimuth
    ledc_update_duty(LEDC_MODE_USED, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_MODE_USED, LEDC_CHANNEL_1);
    
    // Set direction pins to neutral
    gpio_set_level(ACTUATOR_ELEVATION_DIR_PIN, 0);
    gpio_set_level(ACTUATOR_AZIMUTH_DIR_PIN, 0);
}

bool motor_move_to_safe_position(void) {
    ESP_LOGI(TAG, "🏠 Moving to safe storage position");
    
    // Safe position: Panel horizontal (0° elevation), facing east (-90° azimuth)
    // This protects panel from wind and prepares for morning sun
    motor_move_elevation_degrees(0.0f, 30);    // Slow movement to horizontal
    motor_move_azimuth_degrees(-90.0f, 30);    // Slow movement to east
    
    ESP_LOGI(TAG, "✅ Safe position reached - system ready for shutdown");
    return true;
}

bool motor_move_to_reference_position(void) {
    ESP_LOGI(TAG, "📍 Moving to reference calibration position");
    
    // Reference position: True south, local latitude elevation
    float ref_elevation = REFERENCE_ELEVATION_DEG;   // Your local latitude
    float ref_azimuth = REFERENCE_AZIMUTH_DEG;       // True south (180°)
    
    ESP_LOGI(TAG, "Reference position: Az=%.1f° (true south), El=%.1f° (latitude)", 
             ref_azimuth, ref_elevation);
    
    // Move to reference position
    motor_move_elevation_degrees(ref_elevation, 40);
    motor_move_azimuth_degrees(ref_azimuth, 40);
    
    // Update position tracking and mark as calibrated
    current_elevation_degrees = ref_elevation;
    current_azimuth_degrees = ref_azimuth;
    position_calibrated = true;
    
    ESP_LOGI(TAG, "✅ Reference position calibration complete");
    ESP_LOGI(TAG, "System ready for accurate sun tracking operations");
    
    return true;
}