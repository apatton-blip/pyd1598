

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

typedef struct pyd1598_itf {
    int instance_id;
    const struct device* spi_bus;
    struct gpio_dt_spec direct_link_gpio;
} pyd1598_itf_t;

typedef enum op_mode {
    OPERATION_MODES_FORCED_READOUT = 0,
    OPERATION_MODES_INTERRUPT_READOUT = 1,
    OPERATION_MODES_WAKEUP = 2,
    // RESERVED (3)
} operation_modes;

typedef enum sig_src {
    SIGNAL_SOURCE_BPF = 0, // typ. ~8000 count offset. adc: [-8192, 8191]
    SIGNAL_SOURCE_LPF = 1, // adc: [0, 16383]
    // RESERVED (2)
    SIGNAL_SOURCE_TEMPERATURE = 3  // proportional to internal temp. adc: [0, 16383]
} signal_source;

typedef enum hpf_co {
    HPF_CUTOFF_0_4HZ = 0,
    HPF_CUTOFF_0_2HZ = 1
} hpf_cutoff;

typedef enum cm {
    COUNT_MODE_WITH_BPF_SIGN_CHANGE = 0,
    COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE = 1
} count_mode;

typedef void (*pyd1598_isr_safe_cb_t)(const struct device *dev, void *user_data);

int set_config_bypass(const struct device* dev, uint32_t config);

// Sets BPF motion sensitivity [Range: 0-255]
int config_set_target_threshold(const struct device* dev, uint8_t reg);

// set lockout time after motion event. Time = 0.5s + (reg * 0.5s) --> [Range: 0-15]
int config_set_target_blind_time(const struct device* dev, uint8_t reg);

// set motion pulse count threshold criteria. Pulses = 1 + reg --> [Range: 0-3]
int config_set_target_pulse_counter(const struct device* dev, uint8_t reg);

// set moving time window to evaluate pulses. Time = 2s + (reg * 2s) --> [Range: 0-3]
int config_set_target_window_time(const struct device* dev, uint8_t reg);

// set data readout mode
int config_set_target_operation_modes(const struct device* dev, operation_modes mode);

// select ADC source (PIR BPF, PIR LPF, or Temperature)
int config_set_target_signal_source(const struct device* dev, signal_source src);

// set high-pass filter cutoff frequency
int config_set_target_hpf_cutoff(const struct device* dev, hpf_cutoff cutoff);

// set pulse counting logic (with or without zero-crossing)
int config_set_target_count_mode(const struct device* dev, count_mode mode);

uint8_t config_get_current_threshold(const struct device* dev);

uint8_t config_get_current_blind_time(const struct device* dev);

uint8_t config_get_current_pulse_counter(const struct device* dev);

uint8_t config_get_current_window_time(const struct device* dev);

operation_modes config_get_current_operation_modes(const struct device* dev);

signal_source config_get_current_signal_source(const struct device* dev);

hpf_cutoff config_get_current_hpf_cutoff(const struct device* dev);

count_mode config_get_current_count_mode(const struct device* dev);

// atomic build and flash operation; syncs current config with target
int update_current_config(const struct device* dev);

int forced_readout(const struct device* dev);

int set_wakeup_cb(const struct device *dev, pyd1598_isr_safe_cb_t cb, void *user_data);

void print_pyd1598_reading(const struct device* dev);

void print_config_readable(const struct device* dev);