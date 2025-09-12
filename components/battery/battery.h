#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>

/**
 * @brief Initialize battery monitoring ADC
 * 
 * Configures ESP32's ADC1 for battery voltage measurement via voltage divider.
 * Uses 12-bit resolution with 12dB attenuation for 0-3.3V input range.
 * 
 * Hardware Setup:
 * - Battery+ → 10kΩ resistor → GPIO34 (ADC1_CH6) → 10kΩ resistor → GND
 * - This creates 4:1 voltage divider (16.8V max battery → 4.2V → 3.3V ADC)
 * - GPIO34 is input-only, perfect for ADC applications
 * 
 * @note Must be called before other battery functions
 * @note Uses ADC1 (avoid WiFi interference on ADC2)
 */
void battery_init(void);

/**
 * @brief Read current battery voltage
 * 
 * Performs single ADC reading and converts to actual battery voltage.
 * Accounts for voltage divider ratio and ADC reference voltage.
 * 
 * Voltage Calculation:
 * 1. Read 12-bit ADC value (0-4095)
 * 2. Convert to voltage: (raw/4095) × 3.3V × divider_ratio
 * 3. Divider ratio = 4.0 for standard 10kΩ/10kΩ setup
 * 
 * @return Battery voltage in volts (typical range: 10.0-14.4V for 12V system)
 * @note Reading takes ~100μs, safe to call frequently
 * @note Value may fluctuate ±0.1V due to ADC noise and load variations
 */
float battery_read_voltage(void);

/**
 * @brief Check if battery voltage is adequate for operation
 * 
 * Compares current battery voltage against BATTERY_LOW_VOLTAGE threshold.
 * For 12V LiFePO4 systems, typical low voltage is 11.0V (10% SOC).
 * 
 * @return true if battery voltage > BATTERY_LOW_VOLTAGE
 * @return false if battery needs charging or load should be reduced
 * 
 * @note For solar tracker: false result should trigger safe shutdown
 * @note Consider hysteresis to prevent oscillation near threshold
 */
bool battery_is_ok(void);

/**
 * @brief Check if battery is fully charged
 * 
 * Compares current battery voltage against BATTERY_FULL_VOLTAGE threshold.
 * For 12V LiFePO4: 14.4V (charging), 13.6V (float), 12.8V (rested full).
 * 
 * @return true if battery voltage >= BATTERY_FULL_VOLTAGE  
 * @return false if battery has capacity for more charging
 * 
 * @note Useful for solar charge controller logic
 * @note Voltage alone doesn't indicate true SOC - current matters too
 */
bool battery_is_full(void);

#endif