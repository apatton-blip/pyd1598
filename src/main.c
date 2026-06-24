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
    // set_config_bypass(dev, 0x10);
    ret = update_current_config(dev);
    printk("\nconfig build/flash returned: %s (%d)\n\n", ret ? "failure" : "success", ret);
    print_config_readable(dev);
    return ret;
}

int flash_custom_config_2(void){
    int ret;
    set_default_config();
    config_set_target_operation_modes(dev, OPERATION_MODES_WAKEUP);
    config_set_target_threshold(dev, 0);
    config_set_target_pulse_counter(dev, 0);
    // ret = set_config_bypass(dev, 0x02A4211);
    // 0 0010 1010 0100 0011 0001 0001
    ret = ret ? ret : update_current_config(dev);
    printk("\nconfig build/flash returned: %s (%d)\n\n", ret ? "failure" : "success", ret);
    print_config_readable(dev);
    return ret;
}

void read_pyd(void){
    if (forced_readout(dev) == 0){
        k_msleep(3);
        // forced_readout(dev);
        print_pyd1598_reading(dev);
    }
}

void switch_loop(void){
    if (test_setup())
        return 1;
    current_config = CONFIG_1;
    if (flash_custom_config_1())
        return 1;
    print_config_readable(dev);
    int count = 0;
    while (true){
        if ((gpio_pin_get_dt(&button) == 0) && count < 25){
            if (current_config == CONFIG_1 || current_config == CONFIG_2){
                read_pyd();
                count += 1;
                // if (flash_custom_config_1())
                //     return 1;
            }
            else{
                k_msleep(3000);
            }
        }
        else{
            count = 0;
            if (current_config == CONFIG_1){
                printk("Switching to CONFIG_2 (---)\n");
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
        k_msleep(3000);
    }
}


// Simple, fast, inline pseudo-random number generator (Xorshift32)
static inline uint32_t pseudo_rand(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

// Helper function to safely construct a valid 25-bit Forced Readout configuration word
static inline uint32_t pack_pyd1598_config(uint8_t thresh, uint8_t bt, uint8_t pc, uint8_t wt, uint8_t ss, uint8_t hc, uint8_t cm) {
    uint32_t cfg = 0;
    cfg |= ((uint32_t)(thresh & 0xFF)) << 17; // Threshold [24:17]
    cfg |= ((uint32_t)(bt     & 0x0F)) << 13; // Blind Time [16:13]
    cfg |= ((uint32_t)(pc     & 0x03)) << 11; // Pulse Counter [12:11]
    cfg |= ((uint32_t)(wt     & 0x03)) << 9;  // Window Time [10:9]
    cfg |= ((uint32_t)(0      & 0x03)) << 7;  // Op Mode [8:7]: 0 = Forced Readout
    cfg |= ((uint32_t)(ss     & 0x03)) << 5;  // Signal Source [6:5]
    cfg |= ((uint32_t)2)               << 3;  // Reserved [4:3]: Must be dec 2 (10b)
    cfg |= ((uint32_t)(hc     & 0x01)) << 2;  // HPF Cut-Off [2]
    cfg |= ((uint32_t)0)               << 1;  // Reserved [1]: Must be dec 0 (0b)
    cfg |= ((uint32_t)(cm     & 0x01)) << 0;  // Count Mode [0]
    return cfg;
}

void test_serin_optimized(void *dev) {
    int fail = 0;
    int suc = 0;
    uint32_t rand_state = 0xACE1U; // Seed for the PRNG
    int ss_arr[] = {0, 1, 3};      // Valid signal sources (BPF, LPF, Temp)

    // =========================================================================
    // PHASE 1: Coarse Grid Boundary Testing (~3,072 iterations)
    // =========================================================================
    // Sweeping critical boundaries instead of every individual step
    for (int thresh = 0; thresh < 256; thresh += 64) {          // 4 steps (0, 64, 128, 192)
        for (int bt = 0; bt < 16; bt += 4) {                     // 4 steps (0, 4, 8, 12)
            for (int pc = 0; pc < 4; pc++) {                     // All 4 steps
                for (int wt = 0; wt < 4; wt++) {                 // All 4 steps
                    for (int ss = 0; ss < 3; ss++) {             // All 3 valid sources
                        for (int hc = 0; hc < 2; hc++) {         // Both HPF corners
                            for (int cm = 0; cm < 2; cm++) {     // Both counting modes
                                
                                uint32_t cfg = pack_pyd1598_config(thresh, bt, pc, wt, ss_arr[ss], hc, cm);
                                set_config_bypass(dev, cfg); // Direct hardware write
                                int ret = update_current_config(dev);
                                if (ret == 0) suc++; else fail++;
                                printk("TESTS PASSED: %d\nTESTS FAILED: %d\n", suc, fail);
                            }
                        }
                    }
                }
            }
        }
        // System health hook: prevent watchdog resets or thread starvation
        // feed_watchdog(); or k_yield();
    }

    fail = 0;
    suc = 0;

    // =========================================================================
    // PHASE 2: Pseudo-Random Fuzzing Verification (1,000 iterations)
    // =========================================================================
    // Injecting randomized distributions to uncover unexpected bit alignment errors
    for (int i = 0; i < 1000; i++) {
        uint8_t r_thresh = pseudo_rand(&rand_state) % 256;
        uint8_t r_bt     = pseudo_rand(&rand_state) % 16;
        uint8_t r_pc     = pseudo_rand(&rand_state) % 4;
        uint8_t r_wt     = pseudo_rand(&rand_state) % 4;
        uint8_t r_ss     = ss_arr[pseudo_rand(&rand_state) % 3];
        uint8_t r_hc     = pseudo_rand(&rand_state) % 2;
        uint8_t r_cm     = pseudo_rand(&rand_state) % 2;

        uint32_t cfg = pack_pyd1598_config(r_thresh, r_bt, r_pc, r_wt, r_ss, r_hc, r_cm);
        set_config_bypass(dev, cfg); // Direct hardware write
        int ret = update_current_config(dev);

        if (ret == 0) suc++; else fail++;
        
        printk("TESTS PASSED: %d\nTESTS FAILED: %d\n", suc, fail);
    }
    
    printk("HYBRID SENSING TEST COMPLETE\n");
}

void two_config_test(void){
    flash_custom_config_1();
    for (uint64_t t = 0; t < 100; t ++){
        read_pyd();
        k_msleep(15);
    }

    flash_custom_config_2();
    k_msleep(1000);

    // flash_custom_config_1();
    // for (uint64_t t = 0; t < 100; t ++){
    //     read_pyd();
    //     k_msleep(15);
    // }
}

int _main(){
    // Wait for serial terminal to instantiate
    gpio_pin_configure_dt(&button, GPIO_INPUT);

    flash_custom_config_1();
    for (uint64_t t = 0; t < 100; t ++){
        read_pyd();
        k_msleep(15);
    }

    flash_custom_config_2();
    k_msleep(1000);

    // flash_custom_config_1();
    // for (uint64_t t = 0; t < 100; t ++){
    //     read_pyd();
    //     k_msleep(15);
    // }

    // test_serin_optimized(dev);

    // switch_loop();

    // for (uint64_t t = 0; t < 100; t ++){
        // read_pyd();
        // k_msleep(15);
    // }

    // while (true){
    //     read_pyd();
    //     k_msleep(15);
    // }

    while (true){
        k_sleep(K_FOREVER);
    }
    return 0;
}


int main(void){
    init_debug();
    k_msleep(2000);

    // debug_event();
    // debug_event();
    // debug_event();
    _main();
}