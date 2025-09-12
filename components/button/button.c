#include "button.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTON";

// Track button press timing for hold detection
static uint32_t button_press_start_time = 0;
static bool button_was_pressed = false;

esp_err_t button_init(void) {
    // Configure GPIO for button input with internal pull-up
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,     // No interrupts (polling mode)
        .mode = GPIO_MODE_INPUT,            // Input only
        .pin_bit_mask = (1ULL << START_BUTTON_PIN),  // Uses raw number from config.h
        .pull_down_en = GPIO_PULLDOWN_DISABLE,       // No pull-down
        .pull_up_en = GPIO_PULLUP_ENABLE    // Enable internal pull-up (47kΩ typical)
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Start button initialized on GPIO %d (active LOW with pull-up)", 
                 START_BUTTON_PIN);
        
        // Initialize button state tracking
        button_press_start_time = 0;
        button_was_pressed = false;
        
        // Log initial button state for diagnostics
        bool initial_state = button_is_pressed();
        ESP_LOGI(TAG, "Initial button state: %s", 
                 initial_state ? "PRESSED (check wiring!)" : "RELEASED (normal)");
        
        // Warn if button appears stuck pressed on startup
        if (initial_state) {
            ESP_LOGW(TAG, "Button appears pressed on startup - check for short circuit");
        }
        
    } else {
        ESP_LOGE(TAG, "Failed to initialize button on GPIO %d: %s", 
                 START_BUTTON_PIN, esp_err_to_name(ret));
    }
    
    return ret;
}

bool button_is_pressed(void) {
    // Button wired active LOW: pressed = 0V (LOW), released = 3.3V (HIGH)
    // Internal pull-up ensures HIGH when button not pressed
    int gpio_level = gpio_get_level(START_BUTTON_PIN);
    
    // Update button timing tracking for hold detection
    uint32_t current_time = xTaskGetTickCount();
    bool currently_pressed = (gpio_level == 0);
    
    if (currently_pressed && !button_was_pressed) {
        // Button just pressed - record start time
        button_press_start_time = current_time;
    } else if (!currently_pressed && button_was_pressed) {
        // Button just released - reset timing
        button_press_start_time = 0;
    }
    
    button_was_pressed = currently_pressed;
    return currently_pressed;
}

bool button_wait_for_start(void) {
    ESP_LOGI(TAG, "Waiting for start button press...");
    ESP_LOGI(TAG, "Button pin: GPIO %d (connect to GND to activate)", START_BUTTON_PIN);
    
    uint32_t wait_start = xTaskGetTickCount();
    
    // Phase 1: Wait for initial button press
    // Poll every 50ms to balance responsiveness vs CPU usage
    while (!button_is_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Optional: Log waiting status every 10 seconds for debugging
        uint32_t wait_time = xTaskGetTickCount() - wait_start;
        if (wait_time % pdMS_TO_TICKS(10000) == 0) {
            ESP_LOGD(TAG, "Still waiting for button press... (%lu seconds)", 
                     wait_time / pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "Button press detected - debouncing...");
    
    // Phase 2: Debounce the press
    // Wait additional time to ensure press is stable (not mechanical bounce)
    vTaskDelay(pdMS_TO_TICKS(50));
    
    if (!button_is_pressed()) {
        // Press was not stable - likely mechanical bounce or EMI glitch
        ESP_LOGW(TAG, "Button press was unstable (bounce/glitch) - ignoring");
        return false;
    }
    
    ESP_LOGI(TAG, "Button press confirmed! Waiting for release...");
    
    // Phase 3: Wait for button release  
    // This ensures user completes the full press-release cycle
    uint32_t press_duration = 0;
    while (button_is_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        press_duration += 50;
        
        // Log if button held for extended time (useful for hold-to-test features)
        if (press_duration % 1000 == 0) {  // Every second
            ESP_LOGD(TAG, "Button held for %lu seconds", press_duration / 1000);
        }
    }
    
    ESP_LOGI(TAG, "Button released after %lu ms", press_duration);
    
    // Phase 4: Debounce the release
    // Ensure release is stable before confirming operation
    vTaskDelay(pdMS_TO_TICKS(50));
    
    if (button_is_pressed()) {
        // Release was not stable
        ESP_LOGW(TAG, "Button release was unstable - operation cancelled");
        return false;
    }
    
    ESP_LOGI(TAG, "Button operation complete - beginning system operations");
    return true;
}

bool button_held_for_duration(uint32_t hold_time_ms) {
    // Check if button is currently pressed
    if (!button_is_pressed()) {
        return false;
    }
    
    // Check if we have valid press start time
    if (button_press_start_time == 0) {
        return false;
    }
    
    // Calculate how long button has been held
    uint32_t current_time = xTaskGetTickCount();
    uint32_t hold_duration = (current_time - button_press_start_time) * portTICK_PERIOD_MS;
    
    return hold_duration >= hold_time_ms;
}