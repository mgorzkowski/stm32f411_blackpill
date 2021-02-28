#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#define SYSTEM_HSE_FREQ     25000000ul              // 25 MHz
#define SYSTEM_AHB_FREQ     96000000ul              // 96 MHz
#define SYSTEM_APB1_FREQ    (SYSTEM_AHB_FREQ / 2)   // 48 MHz
#define SYSTEM_APB2_FREQ    (SYSTEM_AHB_FREQ)       // 96 MHz
#define SYSTEM_FREQ         SYSTEM_AHB_FREQ


typedef enum {
    SYSTEM_SUCCESS,
    SYSTEM_ERROR
} system_status;

typedef enum {
    SYSTEM_BLINKRATE_FAST,
    SYSTEM_BLINKRATE_MODERATE,
    SYSTEM_BLINKRATE_SLOW,
    SYSTEM_BLINKRATE_MAX   // invalid value
} system_blinkrate;

system_status System_Initialize(void);
system_status System_Blink(uint32_t times, system_blinkrate rate);
void System_StaySafeState(void);
void System_Log(char message[]);
void System_WaitForever(void);
void System_Delay(uint32_t ms);

#endif // __SYSTEM_H__