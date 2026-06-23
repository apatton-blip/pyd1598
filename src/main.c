#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "pyd1598/pyd1598.h"
#include <stdbool.h>

static const struct device* const dev = DEVICE_DT_GET(DT_NODELABEL(pyd1598));

#define BUTN0_NODE DT_ALIAS(sw3)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTN0_NODE, gpios);

extern void init_debug(void);
extern void debug_event(void);

enum config{
    CONFIG_1,
    CONFIG_2
};

enum config current_config = CONFIG_1;

struct k_work work_on_wakeup_handle;

void work_on_wakeup(struct k_work handle){
    printk("Thread has awoken\nThread has slept.\n");
}

void on_wake_up(struct device* dev, struct gpio_callback* cb){
    k_work_submit(&work_on_wakeup_handle);
}

int test_setup(void){
    printk("Checking sensor driver initialization status...\n");
    int ret = 0;
    if (dev == NULL || !device_is_ready(dev)) {
        ret = 1;
    }
    printk("setup returned: %s (%d)\n", ret ? "failure" : "success", ret);
    return ret;
}

void set_default_config(void){
    config_set_target_threshold(dev, 21);
    config_set_target_blind_time(dev, 2);
    config_set_target_pulse_counter(dev, 0);
    config_set_target_window_time(dev, 1);
    config_set_target_operation_modes(dev, OPERATION_MODES_FORCED_READOUT);
    config_set_target_signal_source(dev, SIGNAL_SOURCE_BPF);
    config_set_target_hpf_cutoff(dev, HPF_CUTOFF_0_4HZ);
    config_set_target_count_mode(dev, COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE);
}

int flash_custom_config_1(void){
    int ret;
    set_default_config();
    ret = update_current_config(dev);
    printk("\nconfig build/flash returned: %s (%d)\n\n", ret ? "failure" : "success", ret);
    return ret;
}

int flash_custom_config_2(void){
    int ret;
    set_default_config();
    ret = config_set_target_operation_modes(dev, OPERATION_MODES_WAKEUP);
    // ret = set_config_bypass(dev, 0x02A4211);
    // 0 0010 1010 0100 0011 0001 0001
    ret = ret ? ret : update_current_config(dev);
    printk("\nconfig build/flash returned: %s (%d)\n\n", ret ? "failure" : "success", ret);
    return ret;
}

void read_pyd(void){
    if (forced_readout(dev) == 0){
        print_pyd1598_reading(dev);
    }
}

int _main(){
    // Wait for serial terminal to instantiate
    gpio_pin_configure_dt(&button, GPIO_INPUT);

    if (test_setup())
        return 1;
    current_config = CONFIG_1;
    if (flash_custom_config_1())
    return 1;
    print_config_readable(dev);
    int count = 0;
    while (true){
        if ((gpio_pin_get_dt(&button) == 0) && count < 25){
            if (current_config == CONFIG_1){
                read_pyd();
                count += 1;
            }
            else{
                k_msleep(30);
            }
        }
        else{
            count = 0;
            if (current_config == CONFIG_1){
                printk("Switching to CONFIG_2 (WAKEUP)\n");
                current_config = CONFIG_2;
                if (flash_custom_config_2())
                    return 1;
                print_config_readable(dev);
            }
            else {
                printk("Switching to CONFIG_1 (FORCED)\n");
                current_config = CONFIG_1;
                if (flash_custom_config_1())
                    return 1;
                print_config_readable(dev);
            }
            k_msleep(500);
        }
        k_msleep(20);
    }

    // for (uint64_t t = 0; t < 25; t ++){
    //     test_read_pyd();
    //     k_msleep(30);
    // }

    // while (true){
    //     k_sleep(K_FOREVER);
    // }
    // return 0;
}




int main(void){
    init_debug();
    k_msleep(2000);
    debug_event();
    _main();
}