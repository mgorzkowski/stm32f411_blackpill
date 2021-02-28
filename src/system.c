#include "product.h"
#include "system.h"

// drivers
#include "drivers/led.h"
#include "drivers/button.h"
#include "drivers/serial.h"

#define IO_SERIAL       SERIAL1
#define LOGGING_SERIAL  IO_SERIAL
#define NOP()           __asm("nop")

#define SERIAL_INPUT_BUFFER_SIZE_IN_BYTES           (1024)

// circular buffer for incomming data
typedef struct {
    uint8_t buffer[SERIAL_INPUT_BUFFER_SIZE_IN_BYTES];
    uint32_t readPointer;
    uint32_t writePointer;
} serialInputBuffer_t;

// private variables
static serialInputBuffer_t serialInputBuffer = { 0 };
static volatile uint32_t delayCounter_ms = 0;

// private functions
static system_status ClockSystemInitialize(void);
static void UserButtonCallback(void);
static void UserSerialCallback(serial_device device, uint8_t data);
static void WriteToBuffer(uint8_t byte);
static bool ReadFromBuffer(uint8_t* byte);

system_status System_Initialize(void)
{
    // clock initialization
    if (SYSTEM_SUCCESS != ClockSystemInitialize()) {
        return SYSTEM_ERROR;
    }

    // configure SysTick (1ns)
    if (0 != SysTick_Config(SYSTEM_FREQ / 1000000)) {
        return SYSTEM_ERROR;
    }
    // select AHB as clock source
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // initialize led driver
    if (LED_SUCCESS != Led_Initialize()) {
        return SYSTEM_ERROR;
    }

    // initialize button driver
    if (BUTTON_SUCCESS != Button_Initialize(BUTTON_MODE_INTERRUPT, UserButtonCallback)) {
        return SYSTEM_ERROR;
    }

    // initialize serial driver for all devices
    for (serial_device serialNo=SERIAL1; serialNo<SERIAL_DEVICE_MAX; serialNo++)
    {
        if (SERIAL_SUCCESS != Serial_Initialize(serialNo, SERIAL_115200, UserSerialCallback)) {
            return SYSTEM_ERROR;
        }
    }

    // serial input buffer initialization
    memset(&serialInputBuffer, 0, sizeof(serialInputBuffer));
    // turn off buffering of input and output
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // blink
    System_Blink(1, SYSTEM_BLINKRATE_MODERATE);

    // log that initialziation ends correctly
    System_Log("System initialized correctly");

    return SYSTEM_SUCCESS;
}

system_status System_Blink(uint32_t times, system_blinkrate rate)
{
    uint32_t delay = 0;

    // input validation
    if (SYSTEM_BLINKRATE_MAX <= rate) {
        return LED_ERROR;
    }

    switch (rate) {
        case SYSTEM_BLINKRATE_FAST:
            delay = 50;
            break;
        case SYSTEM_BLINKRATE_MODERATE:
            delay = 200;
            break;
        case SYSTEM_BLINKRATE_SLOW:
            delay = 1000;
            break;
        default:
            return LED_ERROR;   // should never happen
    }

    while (times--) {
        Led_TurnOn();
        System_Delay(delay);
        Led_TurnOff();
        System_Delay(delay);
    }

    return LED_SUCCESS;
}

void System_StaySafeState(void)
{
    // disable interrupts
    __disable_irq();
    // turn led on to indicate that device is disabled
    Led_TurnOn();
    // infinite loop
    System_WaitForever();
}

void System_Log(char message[])
{
    serial_device serial = LOGGING_SERIAL;

    // send message
    for (uint32_t i=0; message[i] != '\0'; i++) {
        Serial_SendByte(serial, message[i]);
    }

    // send termination sign
    Serial_SendByte(serial, '\r');
    Serial_SendByte(serial, '\n');
}

void System_WaitForever(void)
{
    // set this variable to 'false' if you want to quit this function
    volatile bool stay_here = true;
    while (stay_here) {
        NOP();
    }
}

void System_Delay(uint32_t ms)
{
    for(uint32_t i=0; i<ms; i++) {
        delayCounter_ms = 1000;
        // wait until delay counter is zero
        while (0 != delayCounter_ms);
    }
}

int _read(int fd, char *buf, size_t count)
{
    UNUSED(fd);
    bool received = false;
    size_t counter = 0;

    while (counter<count) {
        do {
            uint8_t* tmp = (uint8_t*)&buf[counter];
            received = ReadFromBuffer(tmp);
        } while (false == received);
#ifdef IBUFSPLIT    // split lines - quit when LF or CR occured
        if (0x0D == buf[counter] || 0x0A == buf[counter]) {
            counter++;
            break;
        }
#endif
        counter++;
    }
    return counter;
}

int _write(int fd, const char *buf, size_t count)
{
    UNUSED(fd);
    uint8_t *vector = (uint8_t *)buf;
    for (size_t i=0; i<count; i++) {
        Serial_SendByte(IO_SERIAL, vector[i]);
    }
    return count;
}

// Private functions

static system_status ClockSystemInitialize(void)
{
    // turn on HSE (high speed external) oscilator clock
    RCC->CR |= RCC_CR_HSEON;

    // regulator voltage scaling output configuration 
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // flash cache, prefetch and latency settings
    FLASH->ACR = FLASH_ACR_DCRST | FLASH_ACR_ICRST;
    FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS);
 
    // set PLL source (HSI/HSE), division factors M, P, Q and manipulation factor N
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;                                         // set PLL source to HSE
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLM_0;   // PLL M factor : 25 -> '11001'
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_6;                        // PLL N factor : 192 -> '11000000'
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ_Msk;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2;                                             // PLL Q factor : 4 -> '100'
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk;                                          // PLL P fastor : 2 -> '00'

    // wait until HSE is ready
    while (0 == (RCC->CR & RCC_CR_HSERDY));

    // enable PLL
    RCC->CR |= RCC_CR_PLLON; 
    while (0 == (RCC->CR & RCC_CR_PLLRDY));

    // Reset AHB, APB1 and APB2 buses (no divided)
    RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
    RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
    // APB1 (low speed peripheral bus) divided by 2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // set PLL as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(RCC_CFGR_SWS_PLL != (RCC->CFGR & RCC_CFGR_SWS_Msk));

    // turn off HSI
    RCC->CR &= ~RCC_CR_HSION;

    // compress cell
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DSB();
    SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;
    while (0 == (SYSCFG->CMPCR & SYSCFG_CMPCR_READY));

    return SYSTEM_SUCCESS;
}

static void UserSerialCallback(serial_device device, uint8_t data)
{
    if (SERIAL1 == device) {
        WriteToBuffer(data);
    } else if (SERIAL2 == device) {
        // just echo
        Serial_SendByte(SERIAL2, data);
    }
}

static void UserButtonCallback(void)
{
    Led_Toggle();
}

static void WriteToBuffer(uint8_t byte)
{
    serialInputBuffer.buffer[serialInputBuffer.writePointer] = byte;
    serialInputBuffer.writePointer = (serialInputBuffer.writePointer + 1) % sizeof(serialInputBuffer.buffer);
    // checking for buffer overflow - after write writePointer is equal readPointer
    if (serialInputBuffer.writePointer == serialInputBuffer.readPointer) {
        // if this occures consider to extend buffer size
        System_StaySafeState();
    }
}

static bool ReadFromBuffer(uint8_t* byte)
{
    if (serialInputBuffer.writePointer == serialInputBuffer.readPointer) {
        return false;
    }
    else {
        *byte = serialInputBuffer.buffer[serialInputBuffer.readPointer];
        serialInputBuffer.readPointer = (serialInputBuffer.readPointer + 1) % sizeof(serialInputBuffer.buffer);
        return true;
    }
}

__attribute__((interrupt))
void SysTick_Handler(void) {
    if (delayCounter_ms > 0) {
        delayCounter_ms--;
    }
}