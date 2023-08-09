#ifndef DELTA_ARM_H
#define DELTA_ARM_H

#include <stdint.h>

/**
 * @brief       Position and orientation data sent/received from xbee (v3)
 *              This is the data that is sent to the delta arm
*/
typedef struct __attribute__((__packed__))
{
    int8_t state;   ///< the drones state
    int8_t claw;    ///< claw (1 = open)
    float x;        ///< x-position in the Optitrack frame
    float y;        ///< y-position in the Optitrack frame
    float z;        ///< z-position in the Optitrack frame
} delta_arm_t;

extern delta_arm_t delta_arm;

#define DELTA_ARM_BAUDRATE 57600
#define DELTA_ARM_TIMEOUT_S 0.5
#define DELTA_ARM_DATA_SIZE sizeof(delta_arm_t)
#define DELTA_ARM_NUM_FRAMING_BYTES 4 ///< 2 start bytes 2 checksum bytes
#define DELTA_ARM_START_BYTE_1 0x7E
#define DELTA_ARM_START_BYTE_2 0x7F
#define DELTA_ARM_PACKET_SIZE DELTA_ARM_DATA_SIZE + DELTA_ARM_NUM_FRAMING_BYTES

/**
 * @brief       Initialize the delta arm
 *
 * @return      0 if successful, -1 if not
*/
int delta_arm_init(void);

/**
 * @brief       Send data to the delta arm
*/
void delta_send_data(void);

#endif