/**
 * <xbee_receive.h>
 *
 * @brief       Functions for connecting to and recieving xbee messages
 *
 * @addtogroup  XbeeReceive
 * @{
 */

#ifndef __XBEE_RECEIVE__
#define __XBEE_RECEIVE__

#include <stdint.h>
#include <rc/math/ring_buffer.h>
#include <rc/math/matrix.h>
#include <rc/math/algebra.h>
#include <rc/math/vector.h>

/**
 * @brief       Possition and orientation data sent/received from xbee (v1)
 */
typedef struct __attribute__((packed)) xbee_packet_v1_t
{
    uint32_t time;  ///< Unique id for the rigid body being described
    float x;        ///< x-position in the Optitrack frame
    float y;        ///< y-position in the Optitrack frame
    float z;        ///< z-position in the Optitrack frame
    float qx;       ///< qx of quaternion
    float qy;       ///< qy of quaternion
    float qz;       ///< qz of quaternion
    float qw;       ///< qw of quaternion
    // float roll;              ///< roll
    // float pitch;             ///< pitch
    // float yaw;               ///< yaw
    uint8_t trackingValid;  ///< (bool) of whether or not tracking was valid (0 or 1)
    uint16_t sm_event;      ///< event (or input) for state machine
    uint8_t deadByte;       ///< This byte doesn't do anything. It's a place holder to preserve backward compatibility (when 'trackingValid' was a 4-byte value)
} xbee_packet_v1_t;

/**
 * @brief       Possition and orientation data sent/received from xbee (v2)
 */
typedef struct __attribute__((packed)) xbee_packet_v2_t
{
    float x;        ///< x-position in the motion capture frame
    float y;        ///< y-position in the motion capture frame
    float z;        ///< z-position in the motion capture frame
    float yaw;      ///< yaw
} xbee_packet_v2_t;

typedef xbee_packet_v1_t xbee_packet_t;

#define OPTI_NUM_FRAMING_BYTES 2                ///< 2 START bytes
#define OPTI_NUM_CHECKSUM_BYTES 2               ///< fletcher16 checksum

#define OPTI_DATA_LENGTH_V1 sizeof(xbee_packet_v1_t)  ///< Actual Packet Being Sent
#define OPTI_PACKET_LENGTH_V1 OPTI_DATA_LENGTH_V1 + OPTI_NUM_FRAMING_BYTES + OPTI_NUM_CHECKSUM_BYTES

#define OPTI_DATA_LENGTH_V2 sizeof(xbee_packet_v2_t)  ///< Actual Packet Being Sent
#define OPTI_PACKET_LENGTH_V2 OPTI_DATA_LENGTH_V2 + OPTI_NUM_FRAMING_BYTES + OPTI_NUM_CHECKSUM_BYTES

#define OPTI_MAX_PACKET_LENGTH OPTI_PACKET_LENGTH_V1
#define OPTI_START_BYTE1 0x81
#define OPTI_START_BYTE2 0xA1
#define MOCAP_DT_SEC .01
#define ALPHA_CUTOFF 0.93
#define DIFF_POINTS 2


int opti_data_length, opti_packet_length;
extern xbee_packet_t xbeeMsg;
extern xbee_packet_v2_t xbeeMsg_v2;
extern int xbee_portID;
extern float xbee_x_dot;
extern float xbee_y_dot;
extern float xbee_z_dot;
extern float xbee_dt;

/**
 * @brief       Xbee initialization function
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_init(const char *xbee_port);

/**
 * @brief       Read message recieved from XBee
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_getData();

void __copy_xbee_msg(unsigned char* dataPacket);

/**
 * @brief       Print current XBee message to stdout
 */
void XBEE_printData();

/**
 * @brief       Returns derivative estimate of buffer points
 */
void __diff_function(rc_ringbuf_t *x_buffer, rc_ringbuf_t *y_buffer, rc_ringbuf_t *z_buffer);

float __diff_function_helper(rc_ringbuf_t *buffer, float prev_velocity, float dt);

#endif /*__XBEE_RECEIVE__ */

/* @} end group XbeeReceive */