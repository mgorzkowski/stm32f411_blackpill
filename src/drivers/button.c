#include <product.h>

#include "button.h"

#define BUTTON_PORT             (GPIOA)
#define BUTTON_PIN              (0)
#define BUTTON_GPIOMODER_MASK   (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1)
#define BUTTON_RCCEN            (RCC_AHB1ENR_GPIOAEN)

// private filds
static button_mode buttonMode = BUTTON_MODE_MAX;
static button_callback buttonCallback = NULL;
static bool isButtonInitialized = false;

// callback is required for iterrupt mode
button_status Button_Initialize(button_mode mode, void (*callback)())
{
    if (BUTTON_MODE_MAX <= mode || true == isButtonInitialized) {
        return BUTTON_ERROR;
    }

    // enable the clock to GPIO
    RCC->AHB1ENR |= BUTTON_RCCEN;
    __DSB();
    // set pin as input
    BUTTON_PORT->MODER &= ~BUTTON_GPIOMODER_MASK;
    // set pull up - button is active low
    BUTTON_PORT->PUPDR |= GPIO_PUPDR_PUPDR0_0;
    // fallthrough

    if (BUTTON_MODE_INTERRUPT == mode) {
        // set port A as the port on line EXTI0
        SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
        // set sensitive to falling edge
        EXTI->FTSR = EXTI_FTSR_TR0;
        // set mask for interrupt
        EXTI->IMR = EXTI_IMR_IM0;
        // enable interrupt
        NVIC_EnableIRQ(EXTI0_IRQn);     
    }

    buttonMode = mode;
    buttonCallback = callback;
    isButtonInitialized = true;

    return BUTTON_SUCCESS;
}

button_status Button_IsPushed(bool *isPushed)
{
    // function is avaliable for initialized button in pooling mode
    if (buttonMode != BUTTON_MODE_POLLING || false == isButtonInitialized) {
        return BUTTON_ERROR;
    }

    // check if the button is pushed
    if (BUTTON_PORT->IDR & (1U << BUTTON_PIN)) {
        *isPushed = false;
    }
    else {
        *isPushed = true;
    }

    return BUTTON_SUCCESS;
}

// call button callback when interrupt
__attribute__((interrupt))
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;
        if (NULL != buttonCallback) {
            buttonCallback();
        }
    }
}
