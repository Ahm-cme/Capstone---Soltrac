#pragma once
#include <stdbool.h>

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ Status LED System                                                     │
    │ - Visual feedback for system state and operations                    │
    │ - Non-blocking patterns running in background task                   │
    │ - Essential for deployed trackers where console access isn't available│
    └───────────────────────────────────────────────────────────────────────┘

    LED Pattern Language:
    - Each mode has a distinctive pattern that's recognizable from a distance
    - Patterns repeat continuously until mode changes
    - Total pattern duration kept reasonable (~2s max) for responsive feedback
    - Brightness/duty cycle optimized for outdoor visibility vs power consumption

    Hardware considerations:
    - Works with both active-high and active-low LED connections
    - GPIO current limit: 12-20mA (sufficient for standard LEDs)
    - For high-power LEDs, add a transistor driver and external resistor
    - Consider reverse polarity protection diode for robust installations
    - LED visibility: choose colors appropriate for daylight viewing (blue/white best)

    Power impact:
    - LED current draw is negligible compared to GPS/motors (~5-20mA vs 100-500mA)
    - Sleep mode turns off LED completely to minimize deep sleep current
    - Tracking mode uses brief flashes to indicate life without wasting power

    Installation tips:
    - Mount LED where it's visible during maintenance but not distracting at night  
    - Use weatherproof LED assemblies for outdoor installations
    - Consider dual-color LED for more information density (not implemented yet)
*/

typedef enum {
    LED_STARTUP,    // Fast blink (250ms on/off): system initializing, not ready yet
    LED_WAITING,    // Solid on: waiting for GPS fix or user input
    LED_TRACKING,   // Slow pulse (200ms on, 1800ms off): normal operation, following sun
    LED_ERROR,      // Rapid blink (100ms on/off): GPS failure, SD error, or other fault
    LED_SLEEP       // Off: system in deep sleep or powered down
} status_led_mode_t;

/*
    Initialize LED GPIO and start background pattern task.

    Sets up the specified GPIO as an output and creates a FreeRTOS task
    to handle LED patterns. The task runs at priority 4 (lower than critical
    system functions but higher than idle).

    Parameters:
    - gpio_num: GPIO pin connected to LED (or LED driver transistor base)
    - active_high: true if LED turns on when GPIO is high, false if active-low

    Common GPIO choices:
    - GPIO2: Built-in LED on many ESP32 boards (but also SD card pin - avoid conflict)
    - GPIO5: General purpose, safe for LED use
    - GPIO25: DAC output, can do PWM but simple digital works fine
    - Avoid: GPIO0,12,15 (boot straps), GPIO6-11 (flash), GPIO34-39 (input only)

    Returns:
    - true: LED initialized successfully and task started
    - false: GPIO configuration failed
*/
bool status_led_init(int gpio_num, bool active_high);

/*
    Change LED pattern mode.

    This function is thread-safe and can be called from any task.
    The pattern change takes effect on the next pattern loop iteration
    (typically within 100-500ms depending on current mode).

    Usage pattern in main application:
    status_led_set_mode(LED_STARTUP);     // During init
    status_led_set_mode(LED_WAITING);     // GPS searching
    status_led_set_mode(LED_TRACKING);    // Normal operation
    status_led_set_mode(LED_ERROR);       // When faults occur
    status_led_set_mode(LED_SLEEP);       // Before deep sleep

    Parameters:
    - mode: desired LED pattern from status_led_mode_t enum
*/
void status_led_set_mode(status_led_mode_t mode);

/*
    Get current LED mode.

    Useful for conditional logic or debugging. For example, switching back 
    to LED_TRACKING after a temporary LED_ERROR condition.

    Returns: current mode (may briefly lag actual LED output by one pattern cycle)
*/
status_led_mode_t status_led_get_mode(void);