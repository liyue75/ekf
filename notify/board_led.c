#include "board_led.h"

void board_led_init(void)
{
    gpio_init(LED_1, GPIO_OUT);
    gpio_init(LED_2, GPIO_OUT);
    gpio_init(LED_3, GPIO_OUT);
    gpio_init(IMU_CAL_SWITCH, GPIO_IN_PU);
}

void led_on(gpio_t pin_num)
{
    gpio_write(pin_num, 1);
}

void led_off(gpio_t pin_num)
{
    gpio_write(pin_num, 0);
}
