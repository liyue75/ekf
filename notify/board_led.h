#ifndef BOARD_LED_H_
#define BOARD_LED_H_

#include "periph/gpio.h"

#define LED_1 GPIO_PIN(3, 1)
#define LED_2 GPIO_PIN(3, 3)
#define LED_3 GPIO_PIN(3, 5)

void board_led_init(void);
void led_on(gpio_t pin_num);
void led_off(gpio_t pin_num);

#endif // BOARD_LED_H_
