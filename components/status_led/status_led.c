#include "status_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Status LED Implementation                                             │
    │                                                                       │
    │ Pattern Design Philosophy:                                            │
    │ - Each pattern should be instantly recognizable and distinct          │
    │ - Timing chosen for outdoor visibility (not too fast to see)         │
    │ - Power-conscious: minimize ON time during battery operation          │
    │ - Robust: continues running even if other system components fail      │
    │                                                                       │
    │ Pattern Timing Breakdown:                                            │
    │ LED_STARTUP  → 250ms on, 250ms off (2Hz blink, attention-getting)    │
    │ LED_WAITING  → Solid on (easy to spot from distance)                 │
    │ LED_TRACKING → 200ms on, 1800ms off (brief life sign, low power)     │
    │ LED_ERROR    → 100ms on, 100ms off (5Hz blink, urgent/error)         │
    │ LED_SLEEP    → Always off (zero power draw)                          │
    │                                                                       │
    │ Thread Safety:                                                        │
    │ - s_mode is volatile and updated atomically (single write)           │
    │ - LED GPIO operations are inherently atomic                           │
    │ - Pattern task reads mode once per loop to avoid mid-pattern changes │
    └───────────────────────────────────────────────────────────────────────┘
*/

#define TAG "STATUS_LED"

// Module state (simple global variables for this single LED)
static int s_led_gpio = -1;                            // GPIO pin number (-1 = not initialized)
static bool s_active_high = true;                      // LED polarity (true = turns on with GPIO high)
static volatile status_led_mode_t s_mode = LED_SLEEP;  // Current pattern mode (volatile for thread safety)

/*
    Set physical LED state accounting for active-high/active-low polarity.
    
    This abstraction layer handles the electrical polarity so the rest of
    the code can think in terms of "LED on/off" rather than "GPIO high/low".
    
    Parameters:
    - on: true = turn LED on, false = turn LED off
*/
static inline void led_write(bool on){
    if (s_led_gpio < 0) return;  // Safety check for uninitialized GPIO
    
    int level = (on ? 1 : 0);
    if (!s_active_high) level = !level;  // Invert for active-low LEDs
    
    gpio_set_level(s_led_gpio, level);
    
    ESP_LOGV(TAG, "LED %s (GPIO%d = %d)", on ? "ON" : "OFF", s_led_gpio, level);
}

/*
    Background task that generates LED patterns.
    
    This task runs continuously at low priority and generates the appropriate
    pattern based on the current s_mode value. Each pattern is implemented
    as a simple state machine with timing based on vTaskDelay().
    
    Task design decisions:
    - Priority 4: lower than critical functions, higher than idle
    - Stack 2048: minimal since we only do GPIO and delays
    - No memory allocation: uses only stack variables for robustness
    - Mode sampling: reads s_mode once per pattern cycle for consistency
    
    Pattern implementation:
    - Each case handles one complete pattern element (on+off or just on)
    - vTaskDelay() yields CPU while maintaining precise timing
    - Patterns repeat indefinitely until mode changes
    
    Error handling:
    - Task cannot exit (would leave LED in undefined state)
    - Invalid modes default to LED_SLEEP pattern
    - GPIO errors are ignored (LED failure shouldn't crash system)
*/
static void pattern_task(void *arg){
    ESP_LOGI(TAG, "LED pattern task started");
    
    for(;;){
        // Sample current mode once per loop to avoid mid-pattern changes
        status_led_mode_t current_mode = s_mode;
        
        switch(current_mode){
            case LED_STARTUP:
                // Fast blink: attention-getting during initialization
                // 2 Hz rate is fast enough to notice but not annoying
                ESP_LOGD(TAG, "Pattern: STARTUP (fast blink)");
                led_write(true);  
                vTaskDelay(pdMS_TO_TICKS(250));
                led_write(false); 
                vTaskDelay(pdMS_TO_TICKS(250));
                break;
                
            case LED_WAITING:
                // Solid on: easy to spot when waiting for GPS fix
                // Brief yield prevents task starvation while staying on
                ESP_LOGD(TAG, "Pattern: WAITING (solid on)");
                led_write(true);  
                vTaskDelay(pdMS_TO_TICKS(200)); // Keep ON but yield CPU
                break;
                
            case LED_TRACKING:
                // Brief pulse: indicates system is alive but conserves battery
                // 10% duty cycle (200ms on, 1800ms off) minimizes power draw
                ESP_LOGD(TAG, "Pattern: TRACKING (slow pulse)");
                led_write(true);  
                vTaskDelay(pdMS_TO_TICKS(200));     // Brief "heartbeat" flash
                led_write(false); 
                vTaskDelay(pdMS_TO_TICKS(1800));    // Long off period for power saving
                break;
                
            case LED_ERROR:
                // Rapid blink: urgent/error indication, clearly different from startup
                // 5 Hz rate is fast enough to convey urgency
                ESP_LOGD(TAG, "Pattern: ERROR (rapid blink)");
                led_write(true);  
                vTaskDelay(pdMS_TO_TICKS(100));
                led_write(false); 
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case LED_SLEEP:
            default:
                // LED off: zero power consumption for deep sleep
                // Also serves as fallback for invalid modes
                if (current_mode != LED_SLEEP) {
                    ESP_LOGW(TAG, "Unknown LED mode %d, defaulting to SLEEP", current_mode);
                }
                ESP_LOGD(TAG, "Pattern: SLEEP (off)");
                led_write(false); 
                vTaskDelay(pdMS_TO_TICKS(500));     // Modest delay to prevent busy loop
                break;
        }
    }
    
    // This should never be reached, but include for completeness
    ESP_LOGE(TAG, "LED pattern task exiting unexpectedly!");
    vTaskDelete(NULL);
}

bool status_led_init(int gpio_num, bool active_high){
    ESP_LOGI(TAG, "Initializing status LED on GPIO%d (active-%s)", 
             gpio_num, active_high ? "high" : "low");
    
    // Store configuration in module globals
    s_led_gpio = gpio_num; 
    s_active_high = active_high;
    
    // Configure GPIO as output with no pull resistors
    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << s_led_gpio,         // Bit mask for this GPIO
        .mode         = GPIO_MODE_OUTPUT,           // Output mode
        .pull_up_en   = GPIO_PULLUP_DISABLE,       // No pull-up (LED provides load)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,     // No pull-down 
        .intr_type    = GPIO_INTR_DISABLE          // No interrupts needed
    };
    
    esp_err_t ret = gpio_config(&io_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO configuration failed: %s", esp_err_to_name(ret));
        s_led_gpio = -1;  // Mark as uninitialized
        return false;
    }
    
    // Initialize LED to off state
    led_write(false);
    ESP_LOGD(TAG, "LED GPIO configured, initial state: OFF");
    
    // Create background pattern task
    BaseType_t task_ret = xTaskCreate(
        pattern_task,           // Task function
        "status_led",           // Task name (for debugging)
        2048,                   // Stack size in words (2KB should be plenty)
        NULL,                   // Task parameters (none needed)
        4,                      // Priority (lower than critical tasks)
        NULL                    // Task handle (we don't need to control it)
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED pattern task");
        return false;
    }
    
    ESP_LOGI(TAG, "Status LED system initialized successfully");
    return true;
}

void status_led_set_mode(status_led_mode_t mode){ 
    // Log mode changes for debugging (helps track system state transitions)
    if (s_mode != mode) {
        const char* mode_names[] = {"STARTUP", "WAITING", "TRACKING", "ERROR", "SLEEP"};
        const char* old_name = (s_mode < 5) ? mode_names[s_mode] : "UNKNOWN";
        const char* new_name = (mode < 5) ? mode_names[mode] : "UNKNOWN";
        
        ESP_LOGI(TAG, "Mode change: %s → %s", old_name, new_name);
        s_mode = mode;
    }
}

status_led_mode_t status_led_get_mode(void){ 
    return s_mode; 
}