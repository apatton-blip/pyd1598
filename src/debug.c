#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec debug0 = GPIO_DT_SPEC_GET(DT_NODELABEL(debug_pin_0), gpios);
static const struct gpio_dt_spec debug1 = GPIO_DT_SPEC_GET(DT_NODELABEL(debug_pin_1), gpios);

void init_debug(void){
        if (gpio_is_ready_dt(&debug0))
        gpio_pin_configure_dt(&debug0, GPIO_OUTPUT_LOW);
    
    if (gpio_is_ready_dt(&debug1))
        gpio_pin_configure_dt(&debug1, GPIO_OUTPUT_LOW);
}
void debug_event(void)
{
    gpio_pin_set_dt(&debug1, 1);
    // gpio_pin_set_dt(&debug0, 1);

    gpio_pin_set_dt(&debug1, 0);
    // gpio_pin_set_dt(&debug0, 0);
    
}