#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include "pyd1598/pyd1598.h"

static const struct device* const dev = DEVICE_DT_GET(DT_NODELABEL(pyd1598));

#define BUTN0_NODE DT_ALIAS(sw3)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTN0_NODE, gpios);

extern void init_debug(void);

int setup(void){
    int ret = 0;
    if (dev == NULL || !device_is_ready(dev))
        return 1;
    init_debug();
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    set_wakeup_cb(dev, wakeup_cb, NULL);
    set_interrupt_readout_cb(dev, interrupt_readout_cb, NULL);
    
    return ret;
}

void flash_camera_config(void){
    config_set_target_threshold(dev, 100);
    config_set_target_blind_time(dev, 0);
    config_set_target_pulse_counter(dev, 1);
    config_set_target_window_time(dev, 1);
    config_set_target_operation_modes(dev, OPERATION_MODES_WAKEUP);
    config_set_target_signal_source(dev, SIGNAL_SOURCE_BPF);
    config_set_target_hpf_cutoff(dev, HPF_CUTOFF_0_4HZ);
    config_set_target_count_mode(dev, COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE);
}

void read_pyd(void){
    if (update_reading(dev) == 0){
        print_pyd1598_reading(dev);
    }
}

void continuous_read(void){
    while (true){
        read_pyd();
        k_msleep(15);
    }
}

void numbered_read(int range){
    for (uint64_t t = 0; t < range; t++){
        read_pyd();
        k_msleep(15);
    }
}

void two_config_test(void){
    while(true){
        flash_custom_config_1();
        numbered_read(50);
    
        flash_custom_config_2();
        numbered_read(10);
    }
}

void wakeup_cb(const struct device* dev, void* user_data){
    config_set_target_operation_modes(dev, OPERATION_MODES_FORCED_READOUT);
    update_current_config(dev);
    print_config_readable(dev);

    numbered_read(20); // accumulate data

    config_set_target_operation_modes(dev, OPERATION_MODES_WAKEUP); // return to sleep
    update_current_config(dev);
    print_config_readable(dev);
}

void interrupt_readout_cb(const struct device* dev, void* user_data){
    read_pyd();
}

int main(void){
    k_msleep(2000); // Wait for serial terminal to instantiate
    int setup_ret = setup();
    printk("setup returned: %s (%d)\n", setup_ret ? "failure" : "success", setup_ret);
    if (setup_ret)
        return 1;

    flash_camera_config();
    update_current_config(dev);
    print_config_readable(dev);

    while (true){
        k_sleep(K_FOREVER);
    }
    return 0;
}