#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
    Module notes (internal):
    - Keep this dead simple. Single global config, no dynamic allocation.
    - We intentionally avoid logging here to keep the path quiet. The caller
      knows when it’s waiting for input and can log context-sensitive messages.
    - Debounce is “press-only”: we require stability when entering pressed, but
      we do not debounce release (makes long-press more responsive).
    - CPU usage: Loops sleep in 10 ms increments; good balance vs responsiveness.
*/

/*
    Internal copy of the configuration.
    We keep a single global button instance for simplicity.
*/
static button_cfg_t s_cfg;

/*
    Helper: convert raw GPIO level to a boolean "pressed" according to active_low.
    - If active_low == true, level 0 means pressed.
    - If active_low == false, level 1 means pressed.
*/
static inline bool level_pressed(int level){
    return s_cfg.active_low ? (level == 0) : (level != 0);
}

/*
    Configure the GPIO as an input with desired pull resistors.
    Note: On pins 34..39 the pull settings are ignored by hardware; use external resistors.
    Returns ESP_OK on success.
*/
esp_err_t button_init(const button_cfg_t *cfg){
    s_cfg = *cfg;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_cfg.gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = s_cfg.pull_up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = s_cfg.pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE   // Polled driver; no ISR
    };
    return gpio_config(&io);
}

/*
    Read the pin and report whether the button is currently pressed.
    This is a raw snapshot (no debounce).
*/
bool button_is_pressed(void){
    int lvl = gpio_get_level(s_cfg.gpio);
    return level_pressed(lvl);
}

/*
    Wait for a debounced press edge:
      1) Ensure we start from the "released" state (prevents auto-trigger if already held).
      2) Wait until the signal indicates "pressed".
      3) Delay for debounce_ms and re-check to confirm stability.

    Implementation details & cautions:
    - We measure timeout relative to the first call. Intermediate sleeps consume
      the budget; good enough for human interaction.
    - If debounce_ms == 0, we still perform the re-check immediately to catch
      extremely short bounces.
    - We poll every ~10 ms. If sdkconfig tickrate changes, adjust if needed.
*/
bool button_wait_for_press(int timeout_ms){
    TickType_t start = xTaskGetTickCount();
    TickType_t tmo   = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // Phase 1: wait until released (edge qualification)
    for(;;){
        if (tmo != portMAX_DELAY && (xTaskGetTickCount() - start) >= tmo) return false;
        if (!button_is_pressed()) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Phase 2: wait for press, then debounce
    for(;;){
        if (tmo != portMAX_DELAY && (xTaskGetTickCount() - start) >= tmo) return false;

        if (button_is_pressed()){
            // Debounce: require stable pressed state for debounce_ms
            vTaskDelay(pdMS_TO_TICKS(s_cfg.debounce_ms));
            if (button_is_pressed()) return true;

            // If it bounced back to released, keep waiting.
            // (No extra delay; the loop delay below handles pacing.)
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
    Wait for a "long press":
      - Block until the button becomes pressed.
      - Measure continuous press duration.
      - If still pressed after hold_ms, succeed; otherwise, fail and return to caller.

    UX notes:
    - This does not “debounce” the initial transition beyond the 10 ms polling pace.
      If needed, add a short fixed debounce here too (not required in practice).
    - For calibration, we intentionally want it to be forgiving: if the user
      slightly jitters but maintains the hold, we still count it.

    Power/latency:
    - 10 ms polling is a sweet spot. If we ever aim for light sleep while waiting
      for input, consider GPIO wake + ISR variant instead.
*/
bool button_wait_for_long_press(int hold_ms, int timeout_ms){
    TickType_t start = xTaskGetTickCount();
    TickType_t tmo   = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    // Wait for initial press
    while (!button_is_pressed()){
        if (tmo != portMAX_DELAY && (xTaskGetTickCount() - start) >= tmo) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Measure continuous hold duration
    TickType_t t0 = xTaskGetTickCount();
    while (button_is_pressed()){
        if (pdTICKS_TO_MS(xTaskGetTickCount() - t0) >= hold_ms) return true; // held long enough
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Released too early
    return false;
}