#include "battery.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"      

// ADC Hardware Configuration
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_6    // GPIO34 (ADC1_CH6) - input only pin
#define ADC_ATTEN          ADC_ATTEN_DB_12   // 12dB attenuation (0-3.3V range)
#define ADC_BITWIDTH       ADC_BITWIDTH_12   // 12-bit resolution (0-4095 values)
#define ADC_REF_V          3.3f              // ESP32 ADC reference voltage
#define ADC_DIVIDER        4.0f              // Voltage divider ratio (R1+R2)/R2

// Battery Monitoring Constants
#define ADC_SAMPLES        10                // Number of samples for averaging
#define ADC_SAMPLE_DELAY   10                // Milliseconds between samples
#define VOLTAGE_FILTER_ALPHA 0.1f           // Low-pass filter coefficient (0.0-1.0)

static const char *TAG = "BAT";
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static float filtered_voltage = 0.0f;       // Filtered voltage for stability
static bool adc_initialized = false;        // Track initialization state

void battery_init(void) {
    ESP_LOGI(TAG, "Initializing battery monitoring on GPIO34 (ADC1_CH6)");
    
    // Configure ADC unit (ADC1 to avoid WiFi interference)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,    // Disable ULP for normal operation
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure ADC channel for battery monitoring
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    
    ret = adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
        return;
    }
    
    ESP_LOGI(TAG, "ADC configured - Channel: %d, Attenuation: 12dB, Resolution: 12-bit", 
             BATTERY_ADC_CHANNEL);
    
    // Initialize filtered voltage with first reading
    int raw_initial;
    ret = adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &raw_initial);
    if (ret == ESP_OK) {
        filtered_voltage = (raw_initial / 4095.0f) * ADC_REF_V * ADC_DIVIDER;
        ESP_LOGI(TAG, "Initial battery voltage: %.2fV (raw: %d)", filtered_voltage, raw_initial);
    } else {
        ESP_LOGW(TAG, "Failed initial voltage reading: %s", esp_err_to_name(ret));
        filtered_voltage = 12.0f; // Safe default for 12V system
    }
    
    adc_initialized = true;
    ESP_LOGI(TAG, "Battery monitoring ready - Voltage divider ratio: %.1f:1", ADC_DIVIDER);
    
    // Log expected voltage range for troubleshooting
    ESP_LOGI(TAG, "Expected voltage range: %.1fV (low) to %.1fV (full)", 
             BATTERY_LOW_VOLTAGE, BATTERY_FULL_VOLTAGE);
}

float battery_read_voltage(void) {
    if (!adc_initialized || adc1_handle == NULL) {
        ESP_LOGE(TAG, "ADC not initialized - call battery_init() first");
        return 0.0f;
    }
    
    int raw_sum = 0;
    int valid_samples = 0;
    
    // Take multiple samples for noise reduction
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw;
        esp_err_t ret = adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &raw);
        
        if (ret == ESP_OK) {
            raw_sum += raw;
            valid_samples++;
        } else {
            ESP_LOGW(TAG, "ADC read failed (sample %d): %s", i, esp_err_to_name(ret));
        }
        
        if (i < ADC_SAMPLES - 1) {
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_DELAY));
        }
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "All ADC samples failed - returning last known voltage");
        return filtered_voltage;
    }
    
    // Calculate average raw value
    float raw_average = (float)raw_sum / valid_samples;
    
    // Convert to actual voltage
    // Formula: V_battery = (ADC_raw / ADC_max) × V_ref × divider_ratio
    float current_voltage = (raw_average / 4095.0f) * ADC_REF_V * ADC_DIVIDER;
    
    // Apply low-pass filter to reduce noise
    // filtered = α × new + (1-α) × old
    filtered_voltage = (VOLTAGE_FILTER_ALPHA * current_voltage) + 
                      ((1.0f - VOLTAGE_FILTER_ALPHA) * filtered_voltage);
    
    ESP_LOGD(TAG, "Battery: %.2fV (raw: %.0f, samples: %d/%d)", 
             filtered_voltage, raw_average, valid_samples, ADC_SAMPLES);
    
    // Periodic detailed logging for diagnostics
    static int log_counter = 0;
    if (++log_counter >= 10) {  // Every 10th reading
        ESP_LOGI(TAG, "Battery voltage: %.2fV (filtered), %.2fV (raw)", 
                 filtered_voltage, current_voltage);
        log_counter = 0;
    }
    
    return filtered_voltage;
}

bool battery_is_ok(void) {
    float voltage = battery_read_voltage();
    bool is_ok = voltage > BATTERY_LOW_VOLTAGE;
    
    if (!is_ok) {
        ESP_LOGW(TAG, "⚠️  Battery voltage LOW: %.2fV (threshold: %.2fV)", 
                 voltage, BATTERY_LOW_VOLTAGE);
        ESP_LOGW(TAG, "Consider stopping motor operations to preserve power");
    }
    
    return is_ok;
}

bool battery_is_full(void) {
    float voltage = battery_read_voltage();
    bool is_full = voltage >= BATTERY_FULL_VOLTAGE;
    
    if (is_full) {
        ESP_LOGI(TAG, "🔋 Battery fully charged: %.2fV (threshold: %.2fV)", 
                 voltage, BATTERY_FULL_VOLTAGE);
    }
    
    return is_full;
}