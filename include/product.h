#ifndef __PRODUCT_H__
#define __PRODUCT_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#if GENERIC_F411CX
    #include "stm32f4xx.h"
#else
    #error "Platform not supported"
#endif

#define UNUSED(x) (void)(x)

#endif // __PRODUCT_H__