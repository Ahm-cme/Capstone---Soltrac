#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize the start button GPIO and internal pull-up
 * 
 * Configures the specified START_BUTTON_PIN as input with internal pull-up
 * resistor enabled. Button is expected to be wired active-LOW (button connects
 * GPIO to GND when pressed).
 * 
 * @return ESP_OK on successful GPIO configuration
 * @return ESP_ERR_INVALID_ARG if pin number is invalid
 * @return ESP_FAIL on GPIO configuration failure
 */
esp_err_t button_init(void);

/**
 * @brief Wait for complete button press-and-release cycle (blocking)
 * 
 * This function blocks until:
 * 1. Button is pressed (GPIO goes LOW)
 * 2. Press is debounced and confirmed stable
 * 3. Button is released (GPIO goes HIGH) 
 * 4. Release is debounced and confirmed stable
 * 
 * Includes 50ms debounce delays to prevent false triggers from
 * mechanical switch bounce. Total minimum execution time is ~100ms
 * for a quick press-release cycle.
 * 
 * @note This function will block indefinitely if button is never pressed
 * @note Consider adding timeout for production use
 * 
 * @return true when complete press-release cycle is detected
 * @return false if press was not stable (bounce/glitch detected)
 */
bool button_wait_for_start(void);

/**
 * @brief Check current button state without blocking
 * 
 * Immediately reads GPIO level and returns current button state.
 * Does NOT perform debouncing - caller responsible for handling
 * mechanical bounce if needed.
 * 
 * @note For debounced operation, call multiple times with delays
 * @note Useful for polling button state in main loops
 * 
 * @return true if button is currently pressed (GPIO LOW)
 * @return false if button is currently released (GPIO HIGH)
 */
bool button_is_pressed(void);

/**
 * @brief Check if button has been held for specified duration
 * 
 * Non-blocking function that checks if button is currently pressed
 * AND has been continuously pressed for at least the specified time.
 * Useful for implementing hold-to-activate features or test modes.
 * 
 * @param hold_time_ms Minimum time button must be held (milliseconds)
 * @return true if button held for specified duration
 * @return false if button not pressed or not held long enough
 */
bool button_held_for_duration(uint32_t hold_time_ms);