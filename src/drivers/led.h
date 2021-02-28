#ifndef __LED_H__
#define __LED_H__

#include <product.h>

typedef enum {
    LED_SUCCESS,
    LED_ERROR,
    LED_NOT_IMPLEMENTED
} led_status;

led_status Led_Initialize(void);
led_status Led_TurnOn(void);
led_status Led_TurnOff(void);
led_status Led_Toggle(void);

#endif /* __LED_H__ */