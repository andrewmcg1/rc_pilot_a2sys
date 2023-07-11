#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>

#include <delta_arm.h>
#include <xbee_receive.h>

#include <rc/uart.h>

#define DELTA_ARM_BUS 1
#define BAUDRATE 57600
#define TIMEOUT_S 0.5

int delta_init()
{
    return rc_uart_init(DELTA_ARM_BUS, BAUDRATE, TIMEOUT_S, 0, 1, 0);
}

void delta_send_data()
{
    uint8_t buffer[48];
    uint8_t other_buffer[46];
    buffer[0] = 0x7E;
    buffer[1] = 0x7F;

    memcpy(other_buffer, &xbeeMsg_v3, 46);

    for (int i = 0; i < 46; i++)
    {
        buffer[i + 2] = other_buffer[i];
    }

    rc_uart_flush(DELTA_ARM_BUS);
    rc_uart_write(DELTA_ARM_BUS, buffer, 48);
}