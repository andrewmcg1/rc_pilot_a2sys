/**
 * <realsense_payload_receive.h>
 *
 * @brief       Functions for connecting to and recieving realsense_payload messages
 *
 * @addtogroup  RealsensePayload
 * @{
 */

#ifndef __REALSENSE_PAYLOAD__
#define __REALSENSE_PAYLOAD__

#include <stdint.h>


/**
 * @brief       Landing Command received from Realsense Payload
 */
typedef struct __attribute__ ((packed)) landing_command_t
{
    uint64_t time_us;   // Timestamp in microseconds
    float x;            // x-position in meters
    float y;            // y-position in meters
    float z;            // z-position in meters
} landing_command_t;


/**
 * @brief       Pose Update received from Realsense Payload (T265)
 */
typedef struct __attribute__ ((packed)) pose_update_t
{
    uint64_t time_us;   // Timestamp in microseconds
    float x;            // x-position in meters
    float y;            // y-position in meters
    float z;            // z-position in meters
    float roll;         // roll angle in radians
    float pitch;        // pitch angle in radians
    float yaw;          // yaw angle in radians
} pose_update_t;

#define RSP_NUM_FRAMING_BYTES 4
#define RSP_NUM_CHECKSUM_BYTES 2
#define RSP_MAX_DATA_LENGTH 50
#define RSP_MAX_PACKET_LENGTH RSP_MAX_DATA_LENGTH + RSP_NUM_FRAMING_BYTES + RSP_NUM_CHECKSUM_BYTES
#define RSP_START_BYTE1 0x1B
#define RSP_START_BYTE2 0xFE

extern landing_command_t rspLandingCommandMsg;
extern pose_update_t rspPoseUpdateMsg;
extern int realsense_portID;

/**
 * @brief       RealSense Payload initialization function
 *
 * @return      0 on success, -1 on failure
 */
int REALSENSE_init(const char *realsense_port);

/**
 * @brief       Read message recieved from RealSense Payload
 *
 * @return      0 on no new data, msgID on new data, -1 on failure
 */
int REALSENSE_getData();


#endif /*__REALSENSE_PAYLOAD__ */

/* @} end group RealsensePayload */