#include <time.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"  

#include "config.h"
#include "types.h"
#include "button.h"
#include "gps.h"
#include "battery.h"
#include "motor_control.h"
#include "solar_calc.h"
#include "tracking.h"

static const char *TAG = "APP";

// LED configuration
#define LED_PIN    2    // Built-in LED on ESP32 WROOM 32E

// LED Status Patterns
typedef enum {
    LED_STATUS_STARTUP,     // 🔴 Fast blinking (250ms) - system initializing
    LED_STATUS_WAITING,     // 🟢 Steady ON - waiting for start button press
    LED_STATUS_TRACKING,    // 🔵 Slow blinking (2s) - actively tracking sun
    LED_STATUS_ERROR,       // 🟠 Rapid blinking (100ms) - error condition
    LED_STATUS_SLEEP        // ⚫ OFF - system entering sleep mode
} led_status_t;

static led_status_t current_led_status = LED_STATUS_STARTUP;

// Initialize LED
static void led_init(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "LED initialized on GPIO%d", LED_PIN);
}

// LED status task with different patterns
static void led_status_task(void *arg)
{
    bool led_state = false;
    uint32_t blink_delay = 250;  // Default to startup pattern
    
    ESP_LOGI(TAG, "LED status task started");
    
    while (1) {
        switch (current_led_status) {
            case LED_STATUS_STARTUP:
                // 🔴 Fast blinking (250ms) - system initializing
                blink_delay = 250;
                led_state = !led_state;
                break;
                
            case LED_STATUS_WAITING:
                // 🟢 Steady ON - waiting for start button press
                led_state = true;  // Always on
                blink_delay = 100;  // Check status frequently but don't change LED
                break;
                
            case LED_STATUS_TRACKING:
                // 🔵 Slow blinking (2s) - actively tracking sun
                blink_delay = 2000;
                led_state = !led_state;
                break;
                
            case LED_STATUS_ERROR:
                // 🟠 Rapid blinking (100ms) - error condition
                blink_delay = 100;
                led_state = !led_state;
                break;
                
            case LED_STATUS_SLEEP:
                // ⚫ OFF - system entering sleep mode
                led_state = false;  // Always off
                blink_delay = 1000;
                break;
        }
        
        // Update LED state
        gpio_set_level(LED_PIN, led_state ? 1 : 0);
        
        // Wait for next update
        vTaskDelay(pdMS_TO_TICKS(blink_delay));
    }
}

// Function to change LED status with logging
static void set_led_status(led_status_t status)
{
    current_led_status = status;
    
    const char* status_descriptions[] = {
        "🔴 Fast blinking - System initializing",
        "🟢 Steady ON - Waiting for start button", 
        "🔵 Slow blinking - Actively tracking sun",
        "🟠 Rapid blinking - Error condition",
        "⚫ OFF - System entering sleep mode"
    };
    
    ESP_LOGI(TAG, "LED Status: %s", status_descriptions[status]);
}

static void wait_for_start_button(void) {
    ESP_LOGI(TAG, "Ready for operation - press start button to begin tracking");
    set_led_status(LED_STATUS_WAITING);
    button_wait_for_start();
}

void app_main(void) {
    ESP_LOGI(TAG, "=== SOLTRAC GPS SOLAR TRACKER ===");
    
    // Initialize LED first for immediate visual feedback
    led_init();
    
    // Start LED status task
    xTaskCreate(led_status_task, "led_status", 2048, NULL, 4, NULL);
    
    // Start with startup pattern
    set_led_status(LED_STATUS_STARTUP);
    
    ESP_LOGI(TAG, "Initializing components...");
    
    // Initialize each component with potential error checking
    esp_err_t ret = button_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Button initialization failed!");
        set_led_status(LED_STATUS_ERROR);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Show error for 5 seconds
        return;
    }
    
    gps_init();
    battery_init();
    motor_init();
    tracking_init();
    
    // Check battery level before starting
    if (!battery_is_ok()) {
        float voltage = battery_read_voltage();
        ESP_LOGW(TAG, "⚠️  Low battery voltage: %.2fV", voltage);
        set_led_status(LED_STATUS_ERROR);
        vTaskDelay(pdMS_TO_TICKS(10000));  // Show error for 10 seconds
        set_led_status(LED_STATUS_SLEEP);
        tracking_prepare_sleep();
        return;
    }
    
    ESP_LOGI(TAG, "✅ All systems initialized successfully");
    ESP_LOGI(TAG, "🔋 Battery OK - ready for tracking operations");
    
    // Wait for user to press start button
    wait_for_start_button();
    
    ESP_LOGI(TAG, "🌞 Starting solar tracking operations");
    set_led_status(LED_STATUS_TRACKING);
    
    // Begin tracking operations
    tracking_first_run();
    tracking_loop();
    
    // Prepare for sleep
    ESP_LOGI(TAG, "🌙 Solar tracking complete - preparing for sleep");
    set_led_status(LED_STATUS_SLEEP);
    
    // Give LED time to turn off before sleep
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    tracking_prepare_sleep();
}