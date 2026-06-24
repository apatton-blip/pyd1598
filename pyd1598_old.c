#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "pyd1598.h"
#include <errno.h>

#define SUCCESS 0
#define FAILURE 1

#define LL_HIGH 1
#define LL_LOW 0

// parameters defined in pyd1598 datasheet
// SERIN
#define DATA_CLK_LOW_TIME_US 1
#define DATA_CLK_HIGH_TIME_US 1
#define DATA_IN_HOLD_TIME_US 80
#define DATA_LOAD_TIME_US 650

// DIRECT-LINK
#define NUM_SERIAL_BITS 25
#define NUM_DATA_BITS 40
#define DATA_SETUP_TIME_US 120
#define BIT_TIME_US 13 // will depend on capacitive load of direct link
#define UPDATE_TIME_US 1250
#define CONFIG_UPDATE_TIME_US 2400

// sets reserved bits
#define EMPTY_CONFIG 0x10

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
    
// ROM
/*
pins to be configured in device tree
*/
typedef struct pyd1598_itf {
    int instance_id;
    const struct device* spi_bus;
    struct gpio_dt_spec direct_link_gpio;
} pyd1598_itf_t;


// RAM
/*
configuration and last read datastream
*/
typedef struct pyd1598_buf {
    bool initilaized;

    const struct device* dev;
    struct k_work interrupt_readout_work;
    struct gpio_callback dl_isr_handle;
    pyd1598_isr_safe_cb_t wakeup_mode_cb;
    void* user_data;
    
/*
25-bit config (serial)
    [24:17] Threshold
    [16:13] Blind Time
    [12:11] Pulse Counter
    [10:9]  Window Time
    [8:7]   Operation Modes
    [6:5]   Signal Source
    [4:3]   Reserved
    [2]     HPF Cut-Off
    [1]     Reserved
    [0]     Count Mode
*/
    uint32_t pyd1598_config;
/*
40-bit data stream (direct link)
    [39]    Out of Range
    [38:25] ADC Counts
    [24:0]  Current Config
*/
    uint64_t pyd1589_data_stream;
    
    
} pyd1598_buf_t;

static const struct spi_config spi_cfg = {
    .frequency = 1000000, // 1 MHz (1us resolution)
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave = 0,
    .cs = NULL
};

static void dl_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

int config_set_threshold(const struct device* dev, uint8_t reg){
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~THRESHOLD_BIT_MASK;
    pyd1598_buf->pyd1598_config = pyd1598_buf->pyd1598_config | (uint32_t)reg << THRESHOLD_BIT_SHIFT;
    return SUCCESS;
}

int config_set_blind_time(const struct device* dev, uint8_t reg){
    if (reg > 15){
        printk("Blind Time register must be a value [0, 15]\n");
        return -EINVAL;
    }
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~BLIND_TIME_BIT_MASK;
    pyd1598_buf->pyd1598_config = pyd1598_buf->pyd1598_config | (uint32_t)reg << BLIND_TIME_BIT_SHIFT;
    return SUCCESS;
}

int config_set_pulse_counter(const struct device* dev, uint8_t reg){
    if (reg > 3){
        printk("Pulse Counter register must be a value [0, 3]\n");
        return -EINVAL;
    }
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~PULSE_COUNTER_BIT_MASK;
    pyd1598_buf->pyd1598_config |= (uint32_t)reg << PULSE_COUNTER_BIT_SHIFT;
    return SUCCESS;
}

int config_set_window_time(const struct device* dev, uint8_t reg){
    if (reg > 3){
        printk("Window Time register must be a value [0, 3]\n");
        return -EINVAL;
    }
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~WINDOW_TIME_BIT_MASK;
    pyd1598_buf->pyd1598_config |= (uint32_t)reg << WINDOW_TIME_BIT_SHIFT;
    return SUCCESS;
}

int config_set_operation_mode(const struct device* dev, operation_mode mode){
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~OPERATION_MODES_BIT_MASK;
    pyd1598_buf->pyd1598_config |= mode << OPERATION_MODES_BIT_SHIFT;
    return SUCCESS;
}

int config_set_signal_source(const struct device* dev, signal_source src){
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~SIGNAL_SOURCE_BIT_MASK;
    pyd1598_buf->pyd1598_config |= src << SIGNAL_SOURCE_BIT_SHIFT;
    return SUCCESS;    
}

int config_set_hpf_cutoff(const struct device* dev, hpf_cutoff cutoff){
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~HPF_CUTOFF_BIT_MASK;
    pyd1598_buf->pyd1598_config |= cutoff << HPF_CUTOFF_BIT_SHIFT;
    return SUCCESS;    
}

int config_set_count_mode(const struct device* dev, count_mode mode){
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->pyd1598_config &= ~COUNT_MODE_BIT_MASK;
    pyd1598_buf->pyd1598_config |= mode;
    return SUCCESS;    
}

// Readout procedure to be followed by forced / interrupt pulse
static int _readout_of_bits(const struct device* dev, uint64_t* reading){
    const pyd1598_itf_t* pyd1598_itf = dev->config;
    uint64_t read_buf = 0;
    unsigned int lock = irq_lock(); // disable interrupts for timing sensitive section
    for (int index = NUM_DATA_BITS - 1; index >= 0; index--){
        gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_OUTPUT_LOW);
        k_busy_wait(DATA_CLK_LOW_TIME_US);
        gpio_pin_set_dt(&pyd1598_itf->direct_link_gpio, LL_HIGH);
        k_busy_wait(DATA_CLK_HIGH_TIME_US);
        gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INPUT); // release direct-link (high-impedance)
    
        // acquire and store bit (MSb sent frist)
        k_busy_wait(BIT_TIME_US);
        read_buf = read_buf | ((uint64_t)gpio_pin_get_dt(&pyd1598_itf->direct_link_gpio) << index);
    }
    gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    irq_unlock(lock);
    k_busy_wait(UPDATE_TIME_US);
    gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INPUT); // release direct-link (high-impedance)
    *reading = read_buf;
    return SUCCESS;
}

// Only allowed in Forced Readout Mode -- Does not write to PIR's buffer
static int configcmp(const struct device* dev){
    uint64_t validation_buf;
    const pyd1598_itf_t* pyd1598_itf = dev->config;
    const pyd1598_buf_t* pyd1598_buf = dev->data;
    // initiate read via low->high transition
    gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_OUTPUT_HIGH);
    k_busy_wait(DATA_SETUP_TIME_US);
    _readout_of_bits(dev, &validation_buf);
    
    printk("Expected: %" PRIu32 "\nReceived: %" PRIu32 "\n\n", pyd1598_buf->pyd1598_config, (uint32_t)(validation_buf & CONFIG_BIT_MASK));
    
    return (uint32_t)(validation_buf & CONFIG_BIT_MASK) == pyd1598_buf->pyd1598_config;
}

// // Sets PIR's internal configuration using the serial interface
// static int _write_pyd1598_config(const struct device* dev){
//     const pyd1598_itf_t* itf = dev->config;
//     pyd1598_buf_t* buf = dev->data;

//     k_busy_wait(DATA_LOAD_TIME_US);

//     // direct-link line must be held LOW by the hold system during configuration
//     gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
//     unsigned int lock = irq_lock(); // disable interrupts for timing sensitive section
//     for (int index = NUM_SERIAL_BITS - 1; index >= 0; index--){
//         gpio_pin_set_dt(&itf->serin_gpio, LL_LOW);
//         k_busy_wait(DATA_CLK_LOW_TIME_US);
//         gpio_pin_set_dt(&itf->serin_gpio, LL_HIGH);
//         k_busy_wait(DATA_CLK_HIGH_TIME_US);
//         gpio_pin_set_dt(&itf->serin_gpio, buf->pyd1598_config >> index & 1);
//         k_busy_wait(DATA_IN_HOLD_TIME_US); // hold data signal for minimum time stated in datasheet
//     }
//     gpio_pin_set_dt(&itf->serin_gpio, LL_LOW);
//     irq_unlock(lock);
//     // hold serial 'low' for minimum time stated in datasheet to signal configuration is complete
//     k_busy_wait(DATA_LOAD_TIME_US);
    
//     gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_INPUT); // release direct-link (high-impedance)
//     k_busy_wait(CONFIG_UPDATE_TIME_US);
//     return SUCCESS;
// }

#define MAX_TX_BUFFER_SIZE 500
#define PER_BYTE_TIME_US 8 // 1MHz Clk -> (1 us / 1 bit) * (8 bits / 1 byte) = 8 us / byte
#define CLK_END_HIGH_BYTE 0x7F // 0111 1111
#define CLK_END_LOW_BYTE 0x40 //  0100 0000
// Sets PIR's internal configuration using the serial interface
static int build_config(const struct device* dev){
    #define NUM_HOLD_BYTES DATA_IN_HOLD_TIME_US / PER_BYTE_TIME_US

    const pyd1598_itf_t* itf = dev->config;
    pyd1598_buf_t* buf = dev->data;
    static uint8_t tx_start[18]; memset(tx_start, 0xFF, sizeof(tx_start));
    static uint8_t data_load_buf[DATA_LOAD_TIME_US / PER_BYTE_TIME_US + 1] = {0};
    // will generate a 650us hold
    static uint8_t tx_buf[MAX_TX_BUFFER_SIZE];

    uint16_t tx_buf_index = 0;
    // hold time will be rounded down to a multiple of PER_BYTE_TIME_US
    const uint8_t num_hold_bytes = DATA_IN_HOLD_TIME_US / PER_BYTE_TIME_US;
    // +1 for CLK_END_(*)_BYTE
    if ((NUM_HOLD_BYTES + 1) * NUM_SERIAL_BITS > MAX_TX_BUFFER_SIZE){
        printk("Illegal DATA_IN_HOLD_TIME_US was used. Try [80, 150]");
        return FAILURE;
    }
    for (int serial_index = NUM_SERIAL_BITS - 1; serial_index >= 0; serial_index--){
        if ((buf->pyd1598_config >> serial_index) & 1){
            tx_buf[tx_buf_index++] = CLK_END_HIGH_BYTE;
            for (uint8_t _ = 0; _ < num_hold_bytes; _++){ tx_buf[tx_buf_index++] = 0xFF; }
        }
        else{
            tx_buf[tx_buf_index++] = CLK_END_LOW_BYTE;
            for (uint8_t _ = 0; _ < num_hold_bytes; _++){ tx_buf[tx_buf_index++] = 0x00; }
        }
    }

    
    struct spi_buf spi_buffers[4] = {
        {tx_start, sizeof(tx_start)},
        {data_load_buf, sizeof(data_load_buf)},
        {tx_buf, tx_buf_index},
        {data_load_buf, sizeof(data_load_buf)}
    };

    struct spi_buf_set spi_set_buffers = {
        .buffers = spi_buffers,
        .count = 4
    };

    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_OUTPUT_LOW);
    gpio_pin_configure_dt(&itf->direct_link_gpio, GPIO_INPUT);
    int ret = spi_write(itf->spi_bus, &spi_cfg, &spi_set_buffers);
    k_busy_wait(5000);

    return ret;
}



int update_pyd1598_config(const struct device* dev){
    const pyd1598_itf_t* pyd1598_itf = dev->config;
    pyd1598_buf_t* pyd1598_buf = dev->data;
    operation_mode mode = (pyd1598_buf->pyd1598_config >> OPERATION_MODES_BIT_SHIFT) & OPERATION_MODES_RAW_MASK;
    // config_set_operation_mode(dev, OPERATION_MODE_FORCED_READOUT);
    int ret;
    // (buf->pyd1589_data_stream >> CONFIG_BIT_SHIFT) & CONFIG_RAW_MASK
    // for (int tries = 0; tries < 5; tries++){
        printk("error code: %d\n", build_config(dev));
        ret = configcmp(dev);
        k_busy_wait(2000);
        printk("error code: %d\n", build_config(dev));
        ret = configcmp(dev);
        // if (ret)
        //     break;
    // }
    // config_set_operation_mode(dev, mode);

    // if (!ret)
    //     return FAILURE;

    if (mode != OPERATION_MODE_FORCED_READOUT){
        // for (int tries = 0; tries < 5; tries++){
        //     _write_pyd1598_config(dev);
        // }
        gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    }
    else {
        gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_DISABLE);
    }
    pyd1598_buf->initilaized = true;
    return SUCCESS;
}   

int forced_readout(const struct device* dev){
    const pyd1598_itf_t* pyd1598_itf = dev->config;
    pyd1598_buf_t* pyd1598_buf = dev->data;
    if (!pyd1598_buf->initilaized) { return FAILURE;}
    uint64_t buf;
    // initiate read via low->high transition
    // gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_DISABLE); // added
    gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_OUTPUT_HIGH);
    k_busy_wait(DATA_SETUP_TIME_US);
    int ret = _readout_of_bits(dev, &buf);
    if (!ret){ pyd1598_buf->pyd1589_data_stream = buf; }
    // gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_EDGE_TO_ACTIVE); // added
    return ret;
}

void interrupt_readout_cb(struct k_work* work){
    printk("interrupt has been called!\n");
    pyd1598_buf_t* pyd1598_buf = CONTAINER_OF(work, pyd1598_buf_t, interrupt_readout_work);
    const pyd1598_itf_t* pyd1598_itf = pyd1598_buf->dev->config;
    uint64_t buf;
    gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_DISABLE);
    k_busy_wait(DATA_SETUP_TIME_US);
    if (!_readout_of_bits(pyd1598_buf->dev, &buf)){
        pyd1598_buf->pyd1589_data_stream = buf;
    }
    print_pyd1598_reading(pyd1598_buf->dev);
    gpio_pin_interrupt_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}


void print_pyd1598_reading(const struct device* dev){
    pyd1598_buf_t* buf = dev->data;
    uint16_t raw_adc_counts = (buf->pyd1589_data_stream >> ADC_COUNT_BIT_SHIFT) & ADC_COUNT_RAW_MASK;
    uint8_t signal_source = (buf->pyd1589_data_stream >> SIGNAL_SOURCE_BIT_SHIFT) & SIGNAL_SOURCE_RAW_MASK;
    int16_t adc_counts;
    if (signal_source == SIGNAL_SOURCE_BPF){
        adc_counts = raw_adc_counts;    
        // check if adc counts is signed
        if (raw_adc_counts & 0x2000)
            // prepend signed bits
            adc_counts = raw_adc_counts | 0xC000;
        printk("adc counts: %" PRId16 "\n", adc_counts);
        printk("found config: %" PRIu32 "\n", (uint32_t)(buf->pyd1589_data_stream & CONFIG_RAW_MASK));
        printk("target config: %" PRIu32 "\n\n", buf->pyd1598_config & CONFIG_RAW_MASK);
    }
    else{
        printk("adc counts: %" PRIu16 "\n");
        printk("found config: %" PRIu32 "\n", (uint32_t)(buf->pyd1589_data_stream & CONFIG_RAW_MASK));
        printk("target config: %" PRIu32 "\n\n", buf->pyd1598_config & CONFIG_RAW_MASK);
    }
}

int set_wakeup_cb(const struct device *dev, pyd1598_isr_safe_cb_t cb, void *user_data){
	pyd1598_buf_t *pyd1598_buf = dev->data;
	pyd1598_buf->wakeup_mode_cb = cb;
	pyd1598_buf->user_data = user_data;
	return 0;
}

static void dl_isr_handler(const struct device* port, struct gpio_callback *cb, gpio_port_pins_t pins){
    // recover device inst pointer
    pyd1598_buf_t *pyd1598_buf = CONTAINER_OF(cb, pyd1598_buf_t, dl_isr_handle);
    
    const struct device *dev = pyd1598_buf->dev;
    // const pyd1598_itf_t *pyd_itf = dev->config;
    
    operation_mode mode = (pyd1598_buf->pyd1598_config >> OPERATION_MODES_BIT_SHIFT) & OPERATION_MODES_RAW_MASK;
    
    if (mode == OPERATION_MODE_INTERRUPT_READOUT) {
        k_work_submit(&pyd1598_buf->interrupt_readout_work);
    } 

    else if (mode == OPERATION_MODE_WAKEUP) {
        if (pyd1598_buf->wakeup_mode_cb) {
            pyd1598_buf->wakeup_mode_cb(dev, pyd1598_buf->user_data);
        }
    }
}

// setup pins specified in device tree
static int _setup_pyd1598(const struct device* dev){
    const pyd1598_itf_t* pyd1598_itf = dev->config;
    pyd1598_buf_t* pyd1598_buf = dev->data;
    pyd1598_buf->dev = dev;

    // register data-link callback
    gpio_init_callback(
        &pyd1598_buf->dl_isr_handle,
        dl_isr_handler,
        BIT(pyd1598_itf->direct_link_gpio.pin));
    gpio_add_callback(pyd1598_itf->direct_link_gpio.port, &pyd1598_buf->dl_isr_handle);

    // register interrupt_readout callback
    k_work_init(&pyd1598_buf->interrupt_readout_work, interrupt_readout_cb);
    pyd1598_buf->wakeup_mode_cb = NULL;
    pyd1598_buf->pyd1598_config = EMPTY_CONFIG; // set reserved bits

        
    
    if (!device_is_ready(pyd1598_itf->spi_bus) || !gpio_is_ready_dt(&pyd1598_itf->direct_link_gpio)){
        printk("Serial and / or Direct Link GPIO not ready\n");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&pyd1598_itf->direct_link_gpio, GPIO_INPUT);
    return 0;
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