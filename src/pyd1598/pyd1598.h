#ifndef PYD1598_H
#define PYD1598_H

#include <zephyr/drivers/sensor.h>

/**
 * @brief Operation modes for the PYD1598 sensor.
 */
typedef enum op_mode {
    OPERATION_MODES_FORCED_READOUT = 0,    // Host explicitly initiates communication and reads data.
    OPERATION_MODES_INTERRUPT_READOUT = 1, // Sensor periodically interrupts the host (~16ms) to push data.
    OPERATION_MODES_WAKEUP = 2,            // Sensor only interrupts the host when motion criteria are met.
    // RESERVED (3)
} operation_modes;

/**
 * @brief Signal source for the ADC data stream.
 */
typedef enum sig_src {
    SIGNAL_SOURCE_BPF = 0, // Band-pass filtered. Typ. ~8000 count offset. ADC: [-8192, 8191]
    SIGNAL_SOURCE_LPF = 1, // Low-pass filtered. ADC: [0, 16383]
    // RESERVED (2)
    SIGNAL_SOURCE_TEMPERATURE = 3  // Proportional to internal temp. ADC: [0, 16383]
} signal_source;

/**
 * @brief High-pass filter cutoff frequency.
 */
typedef enum hpf_co {
    HPF_CUTOFF_0_4HZ = 0,
    HPF_CUTOFF_0_2HZ = 1
} hpf_cutoff;

/**
 * @brief Pulse counting logic criteria.
 */
typedef enum cm {
    COUNT_MODE_WITH_BPF_SIGN_CHANGE = 0,   // Pulses are only counted if the signal crosses zero.
    COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE = 1 // Pulses are counted regardless of zero-crossing.
} count_mode;

/**
 * @brief Generic callback signature for PYD1598 interrupts.
 */
typedef void (*pyd_cb)(const struct device *dev, void *user_data);

/**
 * @brief Sets the callback executed when the sensor triggers a motion Wake-Up interrupt.
 * @note This callback only fires if the active operation mode is OPERATION_MODES_WAKEUP.
 */
int set_wakeup_cb(const struct device *dev, pyd_cb cb, void *user_data);

/**
 * @brief Sets the callback executed when the sensor triggers a periodic data interrupt.
 * @note This callback only fires if the active operation mode is OPERATION_MODES_INTERRUPT_READOUT.
 */
int set_interrupt_readout_cb(const struct device *dev, pyd_cb cb, void *user_data);

/**
 * @brief Directly overwrites the staged configuration with a raw 25-bit value.
 * @note Validates that the provided config does not exceed the 25-bit mask and that 
 * reserved bits are correctly set to 0x10. Call update_current_config() to apply.
 */
int set_config_bypass(const struct device* dev, uint32_t config);

/**
 * @brief Sets the target BPF motion sensitivity threshold.
 * @param reg Threshold value [Range: 0-255].
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_threshold(const struct device* dev, uint8_t reg);

/**
 * @brief Sets the target lockout time after a motion event occurs.
 * @param reg Blind time multiplier. Time = 0.5s + (reg * 0.5s) --> [Range: 0-15].
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_blind_time(const struct device* dev, uint8_t reg);

/**
 * @brief Sets the target motion pulse count threshold criteria.
 * @param reg Pulse count. Pulses = 1 + reg --> [Range: 0-3].
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_pulse_counter(const struct device* dev, uint8_t reg);

/**
 * @brief Sets the target moving time window to evaluate pulses.
 * @param reg Window time multiplier. Time = 2s + (reg * 2s) --> [Range: 0-3].
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_window_time(const struct device* dev, uint8_t reg);

/**
 * @brief Sets the target data readout operation mode.
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_operation_modes(const struct device* dev, operation_modes mode);

/**
 * @brief Sets the target ADC signal source (PIR BPF, PIR LPF, or Temperature).
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_signal_source(const struct device* dev, signal_source src);

/**
 * @brief Sets the target high-pass filter cutoff frequency.
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_hpf_cutoff(const struct device* dev, hpf_cutoff cutoff);

/**
 * @brief Sets the target pulse counting logic (with or without zero-crossing).
 * @note This modifies the staged configuration. Call update_current_config() to apply.
 */
int config_set_target_count_mode(const struct device* dev, count_mode mode);

/**
 * @brief Gets the active threshold currently flashed to the device.
 */
uint8_t config_get_current_threshold(const struct device* dev);

/**
 * @brief Gets the active blind time currently flashed to the device.
 */
uint8_t config_get_current_blind_time(const struct device* dev);

/**
 * @brief Gets the active pulse counter currently flashed to the device.
 */
uint8_t config_get_current_pulse_counter(const struct device* dev);

/**
 * @brief Gets the active window time currently flashed to the device.
 */
uint8_t config_get_current_window_time(const struct device* dev);

/**
 * @brief Gets the active operation mode currently flashed to the device.
 */
operation_modes config_get_current_operation_modes(const struct device* dev);

/**
 * @brief Gets the active signal source currently flashed to the device.
 */
signal_source config_get_current_signal_source(const struct device* dev);

/**
 * @brief Gets the active HPF cutoff frequency currently flashed to the device.
 */
hpf_cutoff config_get_current_hpf_cutoff(const struct device* dev);

/**
 * @brief Gets the active count mode currently flashed to the device.
 */
count_mode config_get_current_count_mode(const struct device* dev);

/**
 * @brief Retrieves the parsed ADC reading from the last successful data stream fetch.
 * @note Automatically handles two's complement conversion if the signal source is BPF.
 * Call update_reading() prior to this to fetch fresh data from the sensor.
 */
int32_t get_last_adc_count_reading(const struct device* dev);

/**
 * @brief Retrieves the parsed configuration from the last successful data stream fetch.
 */
int32_t get_last_config_reading(const struct device* dev);

/**
 * @brief Retrieves the Out of Range (OOR) flag from the last successful data stream fetch.
 * @return True if the sensor is in a normal operating state, false if it reset/shorted.
 */
bool get_last_oor_reading(const struct device* dev);

/**
 * @brief Builds the staged configuration, flashes it via SPI, 
 * and verifies the configuration via Direct Link. 
 * @return 0 on success, 1 on failure.
 * @note This MUST be called after any config_set_target_* functions for the settings to take effect.
 */
int update_current_config(const struct device* dev);

/**
 * @brief Initiates a Direct Link read sequence to fetch 40 bits (ADC, Config, and Flags) 
 * into the internal buffer.
 * @return 0 on success, 1 on failure.
 * @note if INTERRUPT_READOUT_ADC_UPDATES or WAKEUP_ADC_UPDATES is enabled,
 * update_reading will be done before the registered callback executes.
 */
int update_reading(const struct device* dev);

/**
 * @brief Prints the ADC counts, calculated voltage, and OOR state of the last reading.
 */
void print_pyd1598_reading(const struct device* dev);

/**
 * @brief Prints a human-readable breakdown of the active (current) configuration.
 */
void print_config_readable(const struct device* dev);

#endif