#include <product.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>

#include "system.h"

// drivers
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/serial.h"

#define COMMAND_SIZE    (256)

// backspace support
int normalize(char command[])
{
    volatile int* ptr = (int*) malloc(8 * sizeof(int));
    int counter = 0;
    int size = strlen(command);
    for (int i=0; i<size; i++) {
        if ('\b' == command[i]) {
            if (i >= 1) {
                counter--;
            }
        } else {
            command[counter] = command[i];
            ptr[counter%8] = command[i];
            counter++;
        }
    }
    command[counter++] = '\0';
    return counter;
}

int main(void)
{
    uint32_t number = 0;
    char command[256];

    // system initialize
    if (SYSTEM_SUCCESS != System_Initialize()) {
        System_StaySafeState();
    }

    // main loop
    while (true) {
        printf(">> ");
        fflush(stdout);
        memset(command, 0, sizeof(command));
        if (NULL != fgets(command, sizeof(command), stdin)) {

            normalize(command);

            if (0 == memcmp(command, "add\n", strlen(command))) {
                printf("after %lu is %lu.\n", number, number+1);
                number++;
            }
            else if (0 == memcmp(command, "sub\n", strlen(command))) {
                printf("before %lu is %lu.\n", number, number-1);
                number--;
            }
            else if (0 == memcmp(command, "ledon\n", strlen(command))) {
                Led_TurnOn();
            }
            else if (0 == memcmp(command, "ledoff\n", strlen(command))) {
                Led_TurnOff();
            }
            else if (0 == memcmp(command, "second\n", strlen(command))) {
                System_Delay(1000);
            }
            else if (0 == memcmp(command, "blink\n", strlen(command))) {
                System_Blink(8, SYSTEM_BLINKRATE_FAST);
            }
            else if (0 == memcmp(command, "sos\n", strlen(command))) {
                System_Blink(3, SYSTEM_BLINKRATE_MODERATE);
                System_Blink(3, SYSTEM_BLINKRATE_SLOW);
                System_Blink(3, SYSTEM_BLINKRATE_MODERATE);
            }
            else if (0 == memcmp(command, "sss\n", strlen(command))) {
                System_StaySafeState();
            }
            else {
                puts("Command not found");
            }
        }
    }

    return 0;
}
