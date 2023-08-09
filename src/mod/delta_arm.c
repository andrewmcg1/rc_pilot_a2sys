#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>

#include <delta_arm.h>
#include <xbee_receive.h>
#include <settings.h>

#include <rc/uart.h>

delta_arm_t delta_arm;

int delta_arm_init(void)
{
    int init = rc_uart_init(settings.delta_arm_bus, DELTA_ARM_BAUDRATE, DELTA_ARM_TIMEOUT_S, 0, 1, 0);
    rc_uart_flush(settings.delta_arm_bus);
    return init;
}

void delta_send_data(void)
{
    uint8_t buffer[DELTA_ARM_PACKET_SIZE];
    buffer[0] = DELTA_ARM_START_BYTE_1;
    buffer[1] = DELTA_ARM_START_BYTE_2;
    memcpy(buffer + 2, &delta_arm, DELTA_ARM_DATA_SIZE);
    uint16_t checksum1 = 0;
    uint16_t checksum2 = 0;
    for (unsigned int i = 0; i < DELTA_ARM_DATA_SIZE; i++)
    {
        checksum1 += buffer[i + 2];
        checksum2 += checksum1;
    }
    checksum2++;
    buffer[DELTA_ARM_PACKET_SIZE - 2] = checksum1;
    buffer[DELTA_ARM_PACKET_SIZE - 1] = checksum2;

    rc_uart_write(settings.delta_arm_bus, buffer, DELTA_ARM_PACKET_SIZE);
}