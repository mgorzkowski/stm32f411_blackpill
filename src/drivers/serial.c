#include <product.h>
#include "system.h"

#include "serial.h"

// USART RCCEN
#define USART1_GPIO_RCCEN       (RCC_AHB1ENR_GPIOAEN)
#define USART2_GPIO_RCCEN       (RCC_AHB1ENR_GPIOAEN)

// port name
#define USART1_PORT             (GPIOA)     // TX=A9 RX=A10
#define USART2_PORT             (GPIOA)     // TX=A2 RX=A3

// USART is alternative function
#define USART1_GPIOMODER_TX     (GPIO_MODER_MODER9_1)
#define USART1_GPIOMODER_RX     (GPIO_MODER_MODER10_1)
#define USART2_GPIOMODER_TX     (GPIO_MODER_MODER2_1)
#define USART2_GPIOMODER_RX     (GPIO_MODER_MODER3_1)

// USART is 7th alternative function -> 0b111
#define USART1_GPIO_AF_TX       (GPIO_AFRH_AFSEL9_0 | GPIO_AFRH_AFSEL9_1 | GPIO_AFRH_AFSEL9_2)
#define USART1_GPIO_AF_RX       (GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL10_1 | GPIO_AFRH_AFSEL10_2)
#define USART2_GPIO_AF_TX       (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2)
#define USART2_GPIO_AF_RX       (GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2)

// output speed set to high speed
#define USART1_GPIO_OSPEED      (GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED10_Msk)
#define USART2_GPIO_OSPEED      (GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk)

typedef struct {
    bool isEnabled;
    serial_callback currentSerialCallback;
    serial_baudRate currentBaudRate;
} serialPriv_t;

// private filds
static serialPriv_t serialPriv[SERIAL_DEVICE_MAX] = { 0 };

// private functions
static USART_TypeDef* GetUSART(serial_device device);

serial_status Serial_Initialize(serial_device device, serial_baudRate baudRate, serial_callback callback)
{
    const uint32_t uartClock = SYSTEM_AHB_FREQ;
    uint32_t usartDivider = 0;
    USART_TypeDef* USART = NULL;

    // device validation
    if (SERIAL_DEVICE_MAX <= device) {
        return SERIAL_ERROR;
    }

    // baud rate validation
    if (SERIAL_BAUDRATE_MAX <= baudRate) {
        return SERIAL_ERROR;
    }

    // calculate divider for usart
    switch (baudRate) {
        case SERIAL_9600:
            usartDivider = uartClock / 9600;
            break;
        case SERIAL_19200:
            usartDivider = uartClock / 19200;
            break;
        case SERIAL_115200:
            usartDivider = uartClock / 115200;
            break;
        default:
            return SERIAL_ERROR; // should never happen
    }

    // set serial's private fields
    if (NULL != callback) {
        serialPriv[device].currentSerialCallback = callback;
    }
    serialPriv[device].currentBaudRate = baudRate;

    // enable USART
    switch (device) {
        case SERIAL1:
            // clock enabling
            RCC->AHB1ENR |= USART1_GPIO_RCCEN;
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
            __DSB();

            // set alternative function
            USART1_PORT->MODER |= USART1_GPIOMODER_TX | USART1_GPIOMODER_RX;
            USART1_PORT->AFR[1] |= USART1_GPIO_AF_TX | USART1_GPIO_AF_RX;

            // set tx and rx to high speed
            USART1_PORT->OSPEEDR |= USART1_GPIO_OSPEED;

            // enable interrupt
            NVIC_EnableIRQ(USART1_IRQn);
            break;

        case SERIAL2:
            // clock enabling
            RCC->AHB1ENR |= USART2_GPIO_RCCEN;
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
            __DSB();

            // set alternative function
            USART2_PORT->MODER |= USART2_GPIOMODER_TX | USART2_GPIOMODER_RX;
            USART2_PORT->AFR[0] |= USART2_GPIO_AF_TX | USART2_GPIO_AF_RX;

            // set tx and rx to high speed
            USART2_PORT->OSPEEDR |= USART2_GPIO_OSPEED;

            // enable interrupt
            NVIC_EnableIRQ(USART2_IRQn);
            break;

        default:
            return SERIAL_ERROR;      // should never happen
    }

    USART = GetUSART(device);

    // set baud rate by setting divider
    USART->BRR = usartDivider;

    // enable usart, interrupts, transmitting and receiving
    USART->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;

    return SERIAL_SUCCESS;
}

void Serial_SendByte(serial_device device, uint8_t byte)
{
    USART_TypeDef* USART = GetUSART(device);
    // wait until previous transmission end data register is empty
    while (0 == (USART->SR & USART_SR_TC) || 0 == (USART->SR & USART_SR_TXE));
    USART->DR = byte;
}

bool Serial_ReceiveByte(serial_device device, uint8_t *byte)
{
    bool retVal = false;
    USART_TypeDef* USART = GetUSART(device);
    // check if the data is ready
    if (USART->SR & USART_SR_RXNE) {
        USART->SR &= ~USART_SR_RXNE;
        *byte = USART->DR;
        retVal = true;
    }
    return retVal;
}

serial_status Serial_SetBaudRate(serial_device device, serial_baudRate baudRate)
{
    // input validation
    if (SERIAL_BAUDRATE_MAX <= baudRate || SERIAL_DEVICE_MAX <= device) {
        return SERIAL_ERROR;
    }

    serialPriv[device].currentBaudRate = baudRate;
    return SERIAL_SUCCESS;
}

serial_status Serial_GetBaudRate(serial_device device, serial_baudRate *baudRate)
{
    if (SERIAL_DEVICE_MAX <= device) {
        return SERIAL_ERROR;
    }

    *baudRate = serialPriv[device].currentBaudRate;
    return SERIAL_SUCCESS;
}

serial_status Serial_SetCallback(serial_device device, serial_callback callback)
{
    if (SERIAL_DEVICE_MAX <= device) {
        return SERIAL_ERROR;
    }

    serialPriv[device].currentSerialCallback = callback;
    return SERIAL_SUCCESS;
}

static USART_TypeDef* GetUSART(serial_device device)
{
    USART_TypeDef* retval = NULL;
    switch (device) {
        case SERIAL1:
            retval = USART1;
            break;
        case SERIAL2:
            retval = USART2;
            break;
        default:
            retval = NULL;  // should never happen
            break;
    }
    return retval;
}

static void SerialInterruptHandler(serial_device device)
{
    uint8_t byte = 0;
    serial_callback callback;
    USART_TypeDef* USART = GetUSART(device);

    if (USART->SR & USART_SR_RXNE) {
        USART->SR &= ~USART_SR_RXNE;
        byte = USART->DR;
        callback = serialPriv[device].currentSerialCallback;
        if (NULL != callback) {
            callback(device, byte);
        }
    }
}

__attribute__((interrupt))
void USART1_IRQHandler(void)
{
    SerialInterruptHandler(SERIAL1);
}

__attribute__((interrupt))
void USART2_IRQHandler(void)
{
    SerialInterruptHandler(SERIAL2);
}