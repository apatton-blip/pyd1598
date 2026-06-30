#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
// #include <errno.h>

#include "pyd1598.h"
#include "params.h"

#define SUCCESS 0
#define FAILURE 1

#define LL_HIGH 1
#define LL_LOW 0

// params defined in PYD 1598 / 7655

// SERIN
#define NUM_SERIAL_BITS 25
#define DATA_CLK_LOW_TIME_US 1
#define DATA_CLK_HIGH_TIME_US 1
#define DATA_IN_HOLD_TIME_US 80
#define DATA_LOAD_TIME_US 650

// DIRECT-LINK
#define NUM_DATA_BITS 40
#define DATA_SETUP_TIME_US 120
#define BIT_TIME_US 8 // will depend on capacitive load of direct link
#define UPDATE_TIME_US 1250
#define CONFIG_UPDATE_TIME_US 2400
#define INT_CLEAR_TIME_US 160

// SERIN SPI CONFIG
#define MAX_TX_BUFFER_SIZE 500
#define PER_BYTE_TIME_US 8 // 1MHz Clk -> (1 us / 1 bit) * (8 bits / 1 byte) = 8 us / byte
#define CLK_END_HIGH_BYTE 0x7F // 0111 1111
#define CLK_END_LOW_BYTE 0x40 //  0100 0000
#define NUM_HOLD_BYTES DATA_IN_HOLD_TIME_US / PER_BYTE_TIME_US // hold time will be rounded down to a multiple of PER_BYTE_TIME_US

#define RESERVED_BIT_MASK 0x1A
#define EMPTY_CONFIG 0x10 // sets reserved bits

#define OUT_OF_RANGE_RAW_MASK     0x1U
#define OUT_OF_RANGE_BIT_SHIFT    39
#define OUT_OF_RANGE_BIT_MASK     (OUT_OF_RANGE_RAW_MASK << OUT_OF_RANGE_BIT_SHIFT)

#define ADC_COUNT_RAW_MASK        0x3FFFU
#define ADC_COUNT_BIT_SHIFT       25
#define ADC_COUNT_BIT_MASK        (ADC_COUNT_RAW_MASK << ADC_COUNT_BIT_SHIFT)

#define CONFIG_RAW_MASK           0x1FFFFFFU
#define CONFIG_BIT_SHIFT          0
#define CONFIG_BIT_MASK           (CONFIG_RAW_MASK << CONFIG_BIT_SHIFT)

#define THRESHOLD_RAW_MASK        0xFFU
#define THRESHOLD_BIT_SHIFT       17
#define THRESHOLD_BIT_MASK        (THRESHOLD_RAW_MASK << THRESHOLD_BIT_SHIFT)

#define BLIND_TIME_RAW_MASK       0xFU
#define BLIND_TIME_BIT_SHIFT      13
#define BLIND_TIME_BIT_MASK       (BLIND_TIME_RAW_MASK << BLIND_TIME_BIT_SHIFT)

#define PULSE_COUNTER_RAW_MASK    0x3U
#define PULSE_COUNTER_BIT_SHIFT   11
#define PULSE_COUNTER_BIT_MASK    (PULSE_COUNTER_RAW_MASK << PULSE_COUNTER_BIT_SHIFT)

#define WINDOW_TIME_RAW_MASK      0x3U
#define WINDOW_TIME_BIT_SHIFT     9
#define WINDOW_TIME_BIT_MASK      (WINDOW_TIME_RAW_MASK << WINDOW_TIME_BIT_SHIFT)

#define OPERATION_MODES_RAW_MASK  0x3U
#define OPERATION_MODES_BIT_SHIFT 7
#define OPERATION_MODES_BIT_MASK  (OPERATION_MODES_RAW_MASK << OPERATION_MODES_BIT_SHIFT)

#define SIGNAL_SOURCE_RAW_MASK    0x3U
#define SIGNAL_SOURCE_BIT_SHIFT   5
#define SIGNAL_SOURCE_BIT_MASK    (SIGNAL_SOURCE_RAW_MASK << SIGNAL_SOURCE_BIT_SHIFT)

#define HPF_CUTOFF_RAW_MASK       0x1U
#define HPF_CUTOFF_BIT_SHIFT      2
#define HPF_CUTOFF_BIT_MASK       (HPF_CUTOFF_RAW_MASK << HPF_CUTOFF_BIT_SHIFT)

#define COUNT_MODE_RAW_MASK       0x1U
#define COUNT_MODE_BIT_SHIFT      0
#define COUNT_MODE_BIT_MASK       (COUNT_MODE_RAW_MASK << COUNT_MODE_BIT_SHIFT)

typedef struct pyd1598_itf {
    int instance_id;
    const struct device* spi_bus;
    struct gpio_dt_spec direct_link_gpio;
} pyd1598_itf_t;

typedef struct pyd1598_buf {
    const struct device* dev; // reverse pointer
    bool config_dirty;

    // sensor data
    uint32_t target_config;
    uint32_t current_config;
    operation_modes mode;
    uint64_t pyd1589_data_stream;
    
    // spi data
    uint8_t tx_buf[MAX_TX_BUFFER_SIZE];
    struct spi_buf spi_buffer[4];
    struct spi_buf_set spi_set_buffers;
    
    // callback data
    bool interrupts_enabled;
    struct gpio_callback dl_gpio_cb;
    struct k_work dl_interrupt_work;

    pyd_cb wakeup_cb;
    void* wakeup_cb_data;

    pyd_cb interrupt_readout_cb;
    void* interrupt_readout_cb_data;
} pyd1598_buf_t;

static const struct spi_config spi_cfg = {
    .frequency = 1000000, // 1 MHz (1us resolution)
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave = 0,
    .cs = {{{{0}}}}
};

static uint8_t DATA_LOAD_BUF[DATA_LOAD_TIME_US / PER_BYTE_TIME_US + 1] = {0};

LOG_MODULE_REGISTER(pyd1598, LOG_LEVEL_INF);

bool is_valid_config(uint32_t config){
    if (config > CONFIG_RAW_MASK){
        LOG_INF("Invalid Config: Configuration must be a value [0x0, 0x1FFFFFF].\n");
        return false;
    }
    if ((config & RESERVED_BIT_MASK) ^ EMPTY_CONFIG){
        LOG_INF("Invalid Config: Reserved are not set.\n");
        return false;
    }
    return true;
}

static int set_target_config(const struct device* dev, uint32_t config){
    pyd1598_buf_t* buf = dev->data;
    if (!is_valid_config(config))
        return FAILURE;
    buf->target_config = config;
    buf->config_dirty = true;
    return SUCCESS;
}

static bool config_has_changed(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    return buf->config_dirty;
}

int set_config_bypass(const struct device* dev, uint32_t config){
    return set_target_config(dev, config);
}

int config_set_target_threshold(const struct device* dev, uint8_t reg){
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~THRESHOLD_BIT_MASK;
    config |= (uint32_t)reg << THRESHOLD_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_blind_time(const struct device* dev, uint8_t reg){
    if (reg > 15){
        LOG_INF("Blind Time register must be a value [0, 15].\n");
        return FAILURE;
    }
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~BLIND_TIME_BIT_MASK;
    config |= (uint32_t)reg << BLIND_TIME_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_pulse_counter(const struct device* dev, uint8_t reg){
    if (reg > 3){
        LOG_INF("Pulse Counter register must be a value [0, 3].\n");
        return FAILURE;
    }
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~PULSE_COUNTER_BIT_MASK;
    config |= (uint32_t)reg << PULSE_COUNTER_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_window_time(const struct device* dev, uint8_t reg){
    if (reg > 3){
        LOG_INF("Window Time register must be a value [0, 3].\n");
        return FAILURE;
    }
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~WINDOW_TIME_BIT_MASK;
    config |= (uint32_t)reg << WINDOW_TIME_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_operation_modes(const struct device* dev, operation_modes mode){
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~OPERATION_MODES_BIT_MASK;
    config |= mode << OPERATION_MODES_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_signal_source(const struct device* dev, signal_source src){
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~SIGNAL_SOURCE_BIT_MASK;
    config |= src << SIGNAL_SOURCE_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_hpf_cutoff(const struct device* dev, hpf_cutoff cutoff){
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~HPF_CUTOFF_BIT_MASK;
    config |= cutoff << HPF_CUTOFF_BIT_SHIFT;
    return set_target_config(dev, config);
}

int config_set_target_count_mode(const struct device* dev, count_mode mode){
    pyd1598_buf_t* buf = dev->data;
    uint32_t config = buf->target_config;
    config &= ~COUNT_MODE_BIT_MASK;
    config |= mode;
    return set_target_config(dev, config);
}

uint8_t config_get_current_threshold(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> THRESHOLD_BIT_SHIFT) & THRESHOLD_RAW_MASK;
}

uint8_t config_get_current_blind_time(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> BLIND_TIME_BIT_SHIFT) & BLIND_TIME_RAW_MASK;
}

uint8_t config_get_current_pulse_counter(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> PULSE_COUNTER_BIT_SHIFT) & PULSE_COUNTER_RAW_MASK;
}

uint8_t config_get_current_window_time(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> WINDOW_TIME_BIT_SHIFT) & WINDOW_TIME_RAW_MASK;
}

operation_modes config_get_current_operation_modes(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> OPERATION_MODES_BIT_SHIFT) & OPERATION_MODES_RAW_MASK;
}

signal_source config_get_current_signal_source(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> SIGNAL_SOURCE_BIT_SHIFT) & SIGNAL_SOURCE_RAW_MASK;
}

hpf_cutoff config_get_current_hpf_cutoff(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> HPF_CUTOFF_BIT_SHIFT) & HPF_CUTOFF_RAW_MASK;
}

count_mode config_get_current_count_mode(const struct device* dev){
    return (((pyd1598_buf_t*)dev->data)->current_config >> COUNT_MODE_BIT_SHIFT) & COUNT_MODE_RAW_MASK;
}

int32_t get_last_adc_count_reading(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    uint16_t adc_counts = (buf->pyd1589_data_stream >> ADC_COUNT_BIT_SHIFT) & ADC_COUNT_RAW_MASK;
    uint8_t signal_source = (buf->pyd1589_data_stream >> SIGNAL_SOURCE_BIT_SHIFT) & SIGNAL_SOURCE_RAW_MASK;
    if (signal_source != SIGNAL_SOURCE_BPF)
        return adc_counts;
    if (adc_counts & 0x2000){ // check if adc counts is signed
        adc_counts = adc_counts | 0xC000; // prepend signed bits
    }
    return (int16_t)adc_counts;
}

int32_t get_last_config_reading(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    return (buf->pyd1589_data_stream & CONFIG_RAW_MASK);
}

bool get_last_oor_reading(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    return (buf->pyd1589_data_stream >> OUT_OF_RANGE_BIT_SHIFT);
}

// configuring DL as input after calling enable_dl_int will corrupt pin.
// preceed with gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_INPUT); */
static void enable_dl_int(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    if (buf->interrupts_enabled)
        return;
    buf->interrupts_enabled = true;
    gpio_add_callback(itf->direct_link_gpio.port, &buf->dl_gpio_cb);
    gpio_pin_interrupt_configure_dt(&itf->direct_link_gpio, GPIO_INT_EDGE_RISING);
}

static void disable_dl_int(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    if (buf->interrupts_enabled){
        buf->interrupts_enabled = false;
        gpio_pin_interrupt_configure_dt(&itf->direct_link_gpio, GPIO_INT_DISABLE);
        gpio_remove_callback_dt(&itf->direct_link_gpio, &buf->dl_gpio_cb);
    }
}

// restore direct-link I/O state depending on operation mode
static void reset_dl(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    if (config_get_current_operation_modes(dev) == OPERATION_MODES_FORCED_READOUT){
        gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
        return;
    }
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_INPUT);
}

// build and load configuration into SPI memory
static int build_config(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    uint16_t tx_buf_index = 0;

    // +1 for CLK_END_(*)_BYTE
    if ((NUM_HOLD_BYTES + 1) * NUM_SERIAL_BITS > MAX_TX_BUFFER_SIZE){
        LOG_WRN("Illegal DATA_IN_HOLD_TIME_US was used. Try [80, 150]");
        return FAILURE;
    }
    for (int serial_index = NUM_SERIAL_BITS - 1; serial_index >= 0; serial_index--){
        bool bit_is_set = (buf->target_config >> serial_index) & 1;
        uint8_t leading_byte = bit_is_set ? CLK_END_HIGH_BYTE : CLK_END_LOW_BYTE;
        uint8_t fill_byte = bit_is_set ? 0xFF : 0x00;
        buf->tx_buf[tx_buf_index++] = leading_byte;
        memset(buf->tx_buf + tx_buf_index, fill_byte, NUM_HOLD_BYTES);
        tx_buf_index += NUM_HOLD_BYTES;
    }
    buf->spi_buffer[1].buf = buf->tx_buf;
    buf->spi_buffer[1].len = tx_buf_index;
    buf->config_dirty = false;
    return SUCCESS;
}

// flash PIR using SPI MOSI, and sync current config with target
static int flash_config(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    const pyd1598_itf_t* itf = dev->config;
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    int ret = spi_write(itf->spi_bus, &spi_cfg, &buf->spi_set_buffers);
    if (ret)
        return ret;
    buf->current_config = buf->target_config;
    k_busy_wait(CONFIG_UPDATE_TIME_US);
    return SUCCESS;
}

// Readout procedure preceeded by forced / interrupt pulse
static int readout_of_bits(const struct device* dev, uint64_t* reading){
    const pyd1598_itf_t* itf = dev->config;
    uint64_t read_buf = 0;
    unsigned int lock = irq_lock(); // disable interrupts for timing sensitive section
    for (int index = NUM_DATA_BITS - 1; index >= 0; index--){
        gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
        k_busy_wait(DATA_CLK_LOW_TIME_US);
        gpio_pin_set_dt(&itf->direct_link_gpio, LL_HIGH);
        k_busy_wait(DATA_CLK_HIGH_TIME_US);
        gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_INPUT); // release direct-link (high-impedance)
        // acquire and store bit (MSb sent frist)
        k_busy_wait(BIT_TIME_US);
        if (gpio_pin_get_dt(&itf->direct_link_gpio))
            read_buf |= 1ULL << index;
    }
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    k_busy_wait(UPDATE_TIME_US);
    irq_unlock(lock);
    reset_dl(dev);
    if (gpio_pin_get_dt(&itf->direct_link_gpio)) // reading likely corrupted
        return FAILURE;
    *reading = read_buf;
    return SUCCESS;
}

static int verify_config(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    uint64_t received;
    // initiate read via low->high transition
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_HIGH);
    k_busy_wait(DATA_SETUP_TIME_US);
    if (readout_of_bits(dev, &received)){}
        return FAILURE;
    return (received & CONFIG_RAW_MASK) == buf->current_config ? SUCCESS : FAILURE;
}

int update_current_config(const struct device* dev){
    if (config_has_changed(dev) && build_config(dev)) // short circuit
        return FAILURE;
    disable_dl_int(dev);
    if (flash_config(dev))
        return FAILURE;
    int ret = verify_config(dev);
    reset_dl(dev);
    if (config_get_current_operation_modes(dev) != OPERATION_MODES_FORCED_READOUT)
        enable_dl_int(dev);
    return ret;
}

int update_reading(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    uint64_t reading;
    // initiate read via low->high transition
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_HIGH);
    k_busy_wait(DATA_SETUP_TIME_US);
    if (readout_of_bits(dev, &reading))
        return FAILURE;
    buf->pyd1589_data_stream = reading;
    return SUCCESS;
}

static void dl_isr(const struct device* port, struct gpio_callback *cb, gpio_port_pins_t pins){
    pyd1598_buf_t* buf = CONTAINER_OF(cb, pyd1598_buf_t, dl_gpio_cb); // recover buffer inst. pointer
    k_work_submit(&buf->dl_interrupt_work); // offload work to gloabl work queue
}

static void clear_wakeup(const struct device* dev){
#if WAKEUP_ADC_UPDATES
    update_reading(dev);
#else
    const pyd1598_itf_t* itf = dev->config;
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    k_busy_wait(INT_CLEAR_TIME_US);
#endif
}

static void clear_interrupt_readout(const struct device* dev){
#if INTERRUPT_READOUT_ADC_UPDATES
    update_reading(dev);
#else
    const pyd1598_itf_t* itf = dev->config;
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    k_busy_wait(INT_CLEAR_TIME_US);
#endif
}

static void dl_interrupt_cb(struct k_work* work){
    pyd1598_buf_t* buf = CONTAINER_OF(work, pyd1598_buf_t, dl_interrupt_work); // recover device inst pointer
    const struct device* dev = buf->dev;
    
    disable_dl_int(dev);

    if (config_get_current_operation_modes(dev) == OPERATION_MODES_INTERRUPT_READOUT){
        clear_interrupt_readout(dev);
        if (buf->interrupt_readout_cb)
            buf->interrupt_readout_cb(dev, buf->interrupt_readout_cb_data);
    }
    else if (config_get_current_operation_modes(dev) == OPERATION_MODES_WAKEUP){
        clear_wakeup(dev);
        if (buf->wakeup_cb) 
            buf->wakeup_cb(dev, buf->wakeup_cb_data);
    }

    if (!buf->interrupts_enabled) { // ensure clean config
        reset_dl(dev);
        if (config_get_current_operation_modes(dev) != OPERATION_MODES_FORCED_READOUT)
        enable_dl_int(dev);
    }
}

static int _setup_pyd1598(const struct device* dev){
    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    // check if port are ready
    if (!device_is_ready(itf->spi_bus) || !gpio_is_ready_dt(&itf->direct_link_gpio)){
        LOG_ERR("Serial and / or Direct Link GPIO not ready\n");
        return FAILURE;
    }

    // setup spi
    buf->spi_buffer[0] = (struct spi_buf){.buf = DATA_LOAD_BUF, .len = sizeof(DATA_LOAD_BUF)};
    buf->spi_buffer[2] = (struct spi_buf){.buf = DATA_LOAD_BUF, .len = sizeof(DATA_LOAD_BUF)};
    buf->spi_set_buffers.buffers = buf->spi_buffer;
    buf->spi_set_buffers.count = 3;

    // register direct-link ISR
    gpio_init_callback(
        &buf->dl_gpio_cb,
        dl_isr,
        BIT(itf->direct_link_gpio.pin));
    k_work_init(&buf->dl_interrupt_work, dl_interrupt_cb);

    buf->dev = dev; // link reverse pointer
    buf->target_config = EMPTY_CONFIG; // set reserved bits
    buf->config_dirty = true;
    
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);

    return SUCCESS;
}

int set_wakeup_cb(const struct device *dev, pyd_cb cb, void *user_data){
	if (!cb)
        return FAILURE;
    pyd1598_buf_t *buf = dev->data;
    buf->wakeup_cb = cb;
	buf->wakeup_cb_data = user_data;
    return SUCCESS;
}

int set_interrupt_readout_cb(const struct device *dev, pyd_cb cb, void *user_data){
	if (!cb)
        return FAILURE;
	pyd1598_buf_t *buf = dev->data;
	buf->interrupt_readout_cb = cb;
	buf->interrupt_readout_cb_data = user_data;
    return SUCCESS;
}

void print_pyd1598_reading(const struct device* dev){
    int32_t adc_counts = get_last_adc_count_reading(dev);
    printk("adc counts: %5" PRId32" - found config: 0x%07"PRIX32" - %s\n",
        adc_counts,
        get_last_config_reading(dev),
        get_last_oor_reading(dev) ? "OK" : "RESET");
}

void print_config_readable(const struct device* dev){
    uint16_t threshold = config_get_current_threshold(dev);
    uint16_t blind_time = config_get_current_blind_time(dev);
    uint8_t pulse_counter = config_get_current_pulse_counter(dev);
    operation_modes op_mode = config_get_current_operation_modes(dev);
    signal_source sig_src = config_get_current_signal_source(dev);
    printk("\n------------ CONFIG: 0x%07X -------------\n", ((pyd1598_buf_t*)dev->data)->current_config);
    printk("Threshold       : %"PRIu16".%"PRIu16" uV\n", threshold * 13 / 2, threshold * 13 % 2 ? 5 : 0);
    printk("Blind Time      : %"PRIu16".%"PRIu16"s\n", blind_time % 2 ? blind_time / 2 + 1 : blind_time / 2, blind_time % 2 ? 0 : 5);
    printk("Pulse Counter   : %"PRIu8" PULS%s\n", 1 + pulse_counter, pulse_counter ? "ES" : "E");
    printk("Window Time     : %"PRIu8"s\n", 2 + 2 * config_get_current_window_time(dev));
    printk("Operation Modes : %s\n", op_mode == OPERATION_MODES_FORCED_READOUT ? "FORCED READOUT" : (op_mode == OPERATION_MODES_INTERRUPT_READOUT ? "INTERRUPT READOUT" : "WAKEUP"));
    printk("Signal Source   : %s\n", sig_src == SIGNAL_SOURCE_BPF ? "BPF" : (sig_src == SIGNAL_SOURCE_LPF ? "LPF" : "TEMPERATURE"));
    printk("HPF Cutoff      : %s Hz\n", config_get_current_hpf_cutoff(dev) ? "0.2" : "0.4");
    printk("Count Mode      : %s SIGN CHANGE\n", config_get_current_count_mode(dev) ? "WITHOUT" : "WITH");
    printk("--------------------------------------------\n\n");
}

#define DT_DRV_COMPAT tactacam_pyd1598

#define INIT_PYD1598(inst)\
    static const pyd1598_itf_t itf_##inst = {\
        .instance_id = inst,\
        .direct_link_gpio = GPIO_DT_SPEC_INST_GET(inst, direct_link_gpios),\
        .spi_bus = DEVICE_DT_GET(DT_INST_PHANDLE(inst, spi_peripheral)),\
    };\
    static pyd1598_buf_t buf_##inst;\
    DEVICE_DT_INST_DEFINE(\
        inst,\
        _setup_pyd1598,\
        NULL,\
        &buf_##inst,\
        &itf_##inst,\
        POST_KERNEL,\
        90,\
        NULL\
    );

DT_INST_FOREACH_STATUS_OKAY(INIT_PYD1598)