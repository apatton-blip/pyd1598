#ifndef FAST_GPIO_H
#define FAST_GPIO_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define CONFIG_SOC_FAMILY_NRF 1

#if defined(CONFIG_SOC_FAMILY_NRF)
#include <hal/nrf_gpio.h>
#endif

/* State structure to hold pre-computed, architecture-specific pin mappings */
typedef struct {
    const struct gpio_dt_spec* dt_spec;
    
    #if defined(CONFIG_SOC_FAMILY_NRF)
    uint32_t hardware_pin;
    #endif
} fast_gpio_t;

/* 1. Initialization (Called once outside time-critical loops) */
/* Pass the port index (e.g., 1) directly into the function */
static inline void fast_gpio_init(fast_gpio_t* fg, const struct gpio_dt_spec* spec, uint32_t port_idx) {
    fg->dt_spec = spec;
    
    #if defined(CONFIG_SOC_FAMILY_NRF)
    fg->hardware_pin = NRF_GPIO_PIN_MAP(port_idx, spec->pin);
    #endif
}

/* 2. Fast Output LOW (Forces direction to Output + Clears pin) */
static inline void fast_gpio_output_low(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        nrf_gpio_pin_clear(fg->hardware_pin);
        nrf_gpio_cfg_output(fg->hardware_pin);
    #else
        gpio_pin_set_dt(fg->dt_spec, 0);
        gpio_pin_configure_dt(fg->dt_spec, GPIO_OUTPUT);
    #endif
}

/* 3. Fast Output HIGH (Forces direction to Output + Sets pin) */
static inline void fast_gpio_output_high(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        nrf_gpio_pin_set(fg->hardware_pin);
        nrf_gpio_cfg_output(fg->hardware_pin);
    #else
        gpio_pin_set_dt(fg->dt_spec, 1);
        gpio_pin_configure_dt(fg->dt_spec, GPIO_OUTPUT);
    #endif
}

/* 4. Fast Set LOW (Alters data register ONLY, does not modify direction configuration) */
static inline void fast_gpio_set_low(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        nrf_gpio_pin_clear(fg->hardware_pin);
    #else
        gpio_pin_set_dt(fg->dt_spec, 0);
    #endif
}

/* 5. Fast Set HIGH (Alters data register ONLY, does not modify direction configuration) */
static inline void fast_gpio_set_high(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        nrf_gpio_pin_set(fg->hardware_pin);
    #else
        gpio_pin_set_dt(fg->dt_spec, 1);
    #endif
}

/* 6. Fast Input (Releases line to high-impedance High-Z with no pull resistors) */
static inline void fast_gpio_make_input(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        nrf_gpio_cfg_input(fg->hardware_pin, NRF_GPIO_PIN_NOPULL);
    #else
        gpio_pin_configure_dt(fg->dt_spec, GPIO_INPUT);
    #endif
}

/* 7. Fast Read (Samples state directly from input buffer register) */
static inline bool fast_gpio_read(fast_gpio_t* fg) {
    #if defined(CONFIG_SOC_FAMILY_NRF)
        return nrf_gpio_pin_read(fg->hardware_pin) > 0;
    #else
        return gpio_pin_get_dt(fg->dt_spec) > 0;
    #endif
}

#endif // FAST_GPIO_H