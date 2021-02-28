#include <product.h>
#include "system.h"

#include "led.h"

#define LED_PORT        (GPIOC)
#define LED_PIN         (13)
#define LED_GPIOMODER   (GPIO_MODER_MODER13_0)
#define LED_RCCEN       (RCC_AHB1ENR_GPIOCEN)

led_status Led_Initialize(void)
{
    // enable the clock to GPIO
    RCC->AHB1ENR |= LED_RCCEN;
    // set pins to be general purpose output
    LED_PORT->MODER |= LED_GPIOMODER;

    return LED_SUCCESS;
}

led_status Led_TurnOn(void)
{
    // turn on the led
    LED_PORT->ODR &= ~(1<<LED_PIN);
    return LED_SUCCESS;
}

led_status Led_TurnOff(void)
{
    // turn off the led
    LED_PORT->ODR |= (1<<LED_PIN);
    return LED_SUCCESS;
}

led_status Led_Toggle(void)
{
    // toggle the led
    LED_PORT->ODR ^= (1<<LED_PIN);
    return LED_SUCCESS;
}
