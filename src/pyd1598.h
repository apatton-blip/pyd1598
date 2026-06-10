#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

typedef enum op_mode {
    OPERATION_MODE_FORCED_READOUT = 0,
    OPERATION_MODE_INTERRUPT_READOUT = 1,
    OPERATION_MODE_WAKEUP = 2,
    // RESERVED (3)
} operation_mode;

typedef enum sig_src {
    SIGNAL_SOURCE_BPF = 0,
    SIGNAL_SOURCE_LPF = 1,
    // RESERVED (2)
    SIGNAL_SOURCE_TEMPERATURE = 3
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

// Sets BPF motion sensitivity [Range: 0-255]
int config_set_threshold(const struct device* dev, uint8_t reg);

// set lockout time after motion event. Time = 0.5s + (reg * 0.5s) --> [Range: 0-15]
int config_set_blind_time(const struct device* dev, uint8_t reg);

// set motion pulse count threshold criteria. Pulses = 1 + reg --> [Range: 0-3]
int config_set_pulse_counter(const struct device* dev, uint8_t reg);

// set moving time window to evaluate pulses. Time = 2s + (reg * 2s) --> [Range: 0-3]
int config_set_window_time(const struct device* dev, uint8_t reg);

// set data readout mode
int config_set_operation_mode(const struct device* dev, operation_mode mode);

// select ADC source (PIR BPF, PIR LPF, or Temperature)
int config_set_signal_source(const struct device* dev, signal_source src);

// set high-pass filter cutoff frequency. [cite: 255]
int config_set_hpf_cutoff(const struct device* dev, hpf_cutoff cutoff);

// set pulse counting logic (with or without zero-crossing)
int config_set_count_mode(const struct device* dev, count_mode mode);

int update_pyd1598_config(const struct device* dev);

int forced_readout(const struct device* dev);

void print_pyd1598_reading(const struct device* dev);