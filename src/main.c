#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "pyd1598.h"
#include <stdbool.h>


#define PYD1598_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(tactacam_pyd1598)
static const struct device* const dev = DEVICE_DT_GET(PYD1598_NODE);

int test_setup(void){
    printk("Checking sensor driver initialization status...\n");
    int ret = 0;
    if (dev == NULL || !device_is_ready(dev)) {
        ret = 1;
    }

    printk("setup returned: %s\n", ret ? "failure" : "success");
    return ret;
}

int test_wakeup_config(void){
    printk("\nstarting config\n-------------------\n");
    config_set_threshold(dev, 43);
    // printk("\n");
    config_set_blind_time(dev, 3);
    // printk("\n");
    config_set_pulse_counter(dev, 1);
    // printk("\n");
    config_set_window_time(dev, 1);
    // printk("\n");
    config_set_operation_mode(dev, OPERATION_MODE_WAKEUP);
    // printk("\n");
    config_set_signal_source(dev, SIGNAL_SOURCE_BPF);
    // printk("\n");
    config_set_hpf_cutoff(dev, HPF_CUTOFF_0_4HZ);
    // printk("\n");
    config_set_count_mode(dev, COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE);
    // printk("\n");
    
    int ret = update_pyd1598_config(dev);
    printk("config returned: %s\n", ret ? "failure" : "success");
    return ret;
    return 0;
}

int test_interrupt_config(void){
    printk("\nstarting config\n-------------------\n");
    config_set_threshold(dev, 0);
    // printk("\n");
    config_set_blind_time(dev, 0);
    // printk("\n");
    config_set_pulse_counter(dev, 1);
    // printk("\n");
    config_set_window_time(dev, 1);
    // printk("\n");
    config_set_operation_mode(dev, OPERATION_MODE_INTERRUPT_READOUT);
    // printk("\n");
    config_set_signal_source(dev, SIGNAL_SOURCE_BPF);
    // printk("\n");
    config_set_hpf_cutoff(dev, HPF_CUTOFF_0_4HZ);
    // printk("\n");
    config_set_count_mode(dev, COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE);
    // printk("\n");
    
    int ret = update_pyd1598_config(dev);
    printk("config returned: %s\n", ret ? "failure" : "success");
    return ret;
    return 0;
}

int test_forced_config(void){
    printk("\nstarting config\n-------------------\n");
    config_set_threshold(dev, 0);
    // printk("\n");
    config_set_blind_time(dev, 0);
    // printk("\n");
    config_set_pulse_counter(dev, 1);
    // printk("\n");
    config_set_window_time(dev, 1);
    // printk("\n");
    config_set_operation_mode(dev, OPERATION_MODE_FORCED_READOUT);
    // printk("\n");
    config_set_signal_source(dev, SIGNAL_SOURCE_BPF);
    // printk("\n");
    config_set_hpf_cutoff(dev, HPF_CUTOFF_0_4HZ);
    // printk("\n");
    config_set_count_mode(dev, COUNT_MODE_WITHOUT_BPF_SIGN_CHANGE);
    // printk("\n");
    
    int ret = update_pyd1598_config(dev);
    // printk("config returned: %s\n", ret ? "failure" : "success");
    return ret;
    return 0;
}

void test_read_pyd(void){
    if (!forced_readout(dev)){
        print_pyd1598_reading(dev);
    }
    
}

struct k_work work_on_wakeup_handle;

void work_on_wakeup(struct k_work handle){
    printk("Thread has awoken\nThread has slept.\n");
}

void on_wake_up(struct device* dev, struct gpio_callback* cb){
    k_work_submit(&work_on_wakeup_handle);
}

int main(void){
    // Wait for serial terminal to instantiate
    k_msleep(2000); 

    if (test_setup())
        return 1;

    // if (test_forced_config())
    //     return 1;
        
    if (test_forced_config())
        return 1;

    while (true){
        test_read_pyd();
        k_msleep(30);
    }

    while (true){
        k_sleep(K_FOREVER);
    }
    return 0;
}