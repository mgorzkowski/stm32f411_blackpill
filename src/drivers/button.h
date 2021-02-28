#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <product.h>

typedef enum {
    BUTTON_SUCCESS,
    BUTTON_ERROR
} button_status;

typedef enum {
    BUTTON_MODE_POLLING,
    BUTTON_MODE_INTERRUPT,
    BUTTON_MODE_MAX
} button_mode;

typedef void (*button_callback)();

button_status Button_Initialize(button_mode mode, void (*callback)());
button_status Button_IsPushed(bool *isPushed);

#endif // __BUTTON_H__