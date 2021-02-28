#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <product.h>

typedef enum {
    SERIAL_SUCCESS,
    SERIAL_ERROR
} serial_status;

typedef enum {
    SERIAL1,
    SERIAL2,
    SERIAL_DEVICE_MAX
} serial_device;

typedef enum {
    SERIAL_9600,
    SERIAL_19200,
    SERIAL_115200,
    SERIAL_BAUDRATE_MAX
} serial_baudRate;

typedef void (*serial_callback)(serial_device device, uint8_t data);

serial_status Serial_Initialize(serial_device device, serial_baudRate baudRate, serial_callback callback);
void Serial_SendByte(serial_device device, uint8_t byte);
bool Serial_ReceiveByte(serial_device device, uint8_t *byte);
serial_status Serial_SetBaudRate(serial_device device, serial_baudRate baudRate);
serial_status Serial_GetBaudRate(serial_device device, serial_baudRate *baudRate);
serial_status Serial_SetCallback(serial_device device, serial_callback callback);


#endif // __SERIAL_H__