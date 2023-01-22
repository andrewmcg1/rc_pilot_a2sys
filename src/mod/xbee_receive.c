/*
 * @file xbee_receive.c
 *
 *  Updated code (Feb 2019)
 *
 *  Using data structure for packets with:
 *  Two start bytes:  0x81, 0xA1
 *  [Not included: Message ID (one byte), Message payload size (one byte) since we only have one
 *  message type] Message data (xbee_packet_t length) Fletcher-16 checksum (two bytes) computed
 *  starting with Message payload size
 *
 *  Note:  This MBin protocol is commonly used on embedded serial devices subject to errors
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>  //one of these two is for memcpy
#include <string.h>
#include <unistd.h>  // read / write

// Below for PRId64
#include <inttypes.h>

#include <rc/time.h>

#include <serial_com.h>
#include <state_estimator.h>
#include <xbee_receive.h>
#include <crc16.h>
#include <settings.h>

#include <math.h>

xbee_packet_t xbeeMsg;  // Defined as extern in xbee_receive.h
xbee_packet_v2_t xbeeMsg_v2; // Defined as extern in xbee_receive.h
int xbee_portID;        // Defined as extern in xbee_receive.h
rc_ringbuf_t x_position_track;  //Defined as extern in xbee_receive.h
rc_ringbuf_t y_position_track;  //Defined as extern in xbee_receive.h
rc_ringbuf_t z_position_track;  //Defined as extern in xbee_receive.h
rc_ringbuf_t time_track; //Defined as extern in xbee_receive.h
// rc_matrix_t diff = RC_MATRIX_INITIALIZER;  //Defined as extern in xbee_receive.h
// rc_vector_t diff_coeff = RC_VECTOR_INITIALIZER;  //Defined as extern in xbee_receive.h
// rc_vector_t rhs = RC_VECTOR_INITIALIZER;  //Defined as extern in xbee_receive.h
float xbee_x_dot = 0.0;  //Defined as extern in xbee_receive.h
float xbee_y_dot = 0.0;  //Defined as extern in xbee_receive.h
float xbee_z_dot = 0.0;  //Defined as extern in xbee_receive.h
float prev_x_velocity = 0.0;
float prev_y_velocity = 0.0;
float prev_z_velocity = 0.0;
uint64_t lastime = 0;
uint64_t thistime = 0;
float xbee_dt; //Defined as extern

int XBEE_init(const char *xbee_port)
{
    int baudRate = 57600;
    xbee_portID = serial_open(xbee_port, baudRate, 0);  // Nonblocking = 0, BLOCKING = 1
    if (xbee_portID == -1)
    {
        printf("Failed to open Serial Port\n");
        return -1;
    }

    x_position_track = rc_ringbuf_empty();
    y_position_track = rc_ringbuf_empty();
    z_position_track = rc_ringbuf_empty();
    time_track = rc_ringbuf_empty();

    if (rc_ringbuf_alloc(&x_position_track, DIFF_POINTS) == -1) {
        printf("Failed to allocate x_position_track ringbuf");
        return -1;
    }
    if (rc_ringbuf_alloc(&y_position_track, DIFF_POINTS) == -1) {
        printf("Failed to allocate y_position_track ringbuf");
        return -1;
    }
    if (rc_ringbuf_alloc(&z_position_track, DIFF_POINTS) == -1) {
        printf("Failed to allocate z_position_track ringbuf");
        return -1;
    }
    if (rc_ringbuf_alloc(&time_track, 2) == -1) {
        printf("Failed to allocate time_track ringbuf");
        return -1;
    }
    // diff = rc_matrix_empty();
    // diff_coeff = rc_vector_empty();
    // rhs = rc_vector_empty();
    // if (rc_matrix_zeros(&diff, DIFF_POINTS, DIFF_POINTS) == -1) {
    //     printf("Failed to allocate difference matrix");
    //     return -1;
    // }
    // if (rc_vector_zeros(&diff_coeff, DIFF_POINTS) == -1) {
    //     printf("Failed to allocate difference matrix");
    //     return -1;
    // }
    // if (rc_vector_zeros(&rhs, DIFF_POINTS) == -1) {
    //     printf("Failed to allocate rhs matrix");
    //     return -1;
    // }

    // rhs.d[1] = 1;

    // for (int i = 0; i < DIFF_POINTS; i++) {
    //     for (int j = 0; j < DIFF_POINTS; j++) {
    //         diff.d[i][j] = (((DIFF_POINTS - 1 - j)^(i))*(-1)^(i));
    //     }
    // }

    // if (rc_algebra_lin_system_solve(diff, rhs, &diff_coeff) == -1) {
    //      printf("Failed to generate difference coefficients");
    //     return -1;
    // }


    // Init packet based on version
    switch (settings.xbee_packet_version)
    {
        case 1:
            opti_data_length = OPTI_DATA_LENGTH_V1;
            opti_packet_length = OPTI_PACKET_LENGTH_V1;
            break;
        case 2:
            opti_data_length = OPTI_DATA_LENGTH_V2;
            opti_packet_length = OPTI_PACKET_LENGTH_V2;

            // Pre-fill some packet values so we don't recopy later
            xbeeMsg.qx = 0;
            xbeeMsg.qy = 0;
            xbeeMsg.time = 0;
            xbeeMsg.trackingValid = 0;
            xbeeMsg.sm_event = 0;
            break;
        default:
            opti_data_length = OPTI_DATA_LENGTH_V1;
            opti_packet_length = OPTI_PACKET_LENGTH_V1;
            break;
    }


    lastime = rc_nanos_since_epoch();
    return 0;
}

// Read messages received from realsense payload
int XBEE_getData()
{
    // Init Packet Pointers
    static unsigned char serialPacket[OPTI_MAX_PACKET_LENGTH];
    static unsigned char* dataPacket = serialPacket + OPTI_NUM_FRAMING_BYTES;
    static unsigned char* ptr = serialPacket;
    static uint8_t chksm0 = 0;
    static uint8_t chksm1 = 0;

    while(read(xbee_portID,ptr,1) > 0) {    
        // 1) if the first Byte is wrong keep looking
        if( (ptr == serialPacket) && (ptr[0] != OPTI_START_BYTE1) ) {
            ptr = serialPacket;
            // printf("First XBee Byte NOT correct!\n");
            continue;
        }

        // 2) if the second Byte is wrong keep looking
        if( (ptr == serialPacket + 1) && (ptr[0] != OPTI_START_BYTE2) ) {
            ptr = serialPacket;
            // printf("Second XBee Byte NOT correct!\n");
            continue;
        }

        // 3) Reset the checksum on the third byte
        if(ptr == serialPacket + 2) {
            chksm0 = 0;
            chksm1 = 0;
        } 

        // 4) Compute the checksum on the data bytes
        if ((ptr >= serialPacket + 2) && (ptr < serialPacket + opti_data_length + OPTI_NUM_FRAMING_BYTES) ) {
            chksm0 += ptr[0];
            chksm1 += chksm0;
            // printf("chksm0: %02x, chksm1: %02x \n", chksm0, chksm1);
        }

        // 5) And then increment the ptr to keep reading in bytes
        ptr++;	

        // 6) Once we have all of the Bytes, Process them!
        if( (ptr-serialPacket) == opti_packet_length) {
            ptr = serialPacket;

            uint16_t checksum_calculated = (uint16_t) ((chksm1 << 8) | chksm0);
            uint16_t checksum_received = 0;
            // uint16_t checksum_with_function = fletcher16(dataPacket, OPTI_DATA_LENGTH);

            memcpy(&checksum_received, &dataPacket[opti_data_length], sizeof(checksum_received));
            // printf("Checksum calculated: %u; Checksum Recieved: %u \n", checksum_calculated, checksum_received);
            
            if (checksum_received == checksum_calculated)
            {
                __copy_xbee_msg(dataPacket);
                state_estimate.xbee_time_received_ns = rc_nanos_since_epoch();
                // printf("Correct checksum!\n");

                rc_ringbuf_insert(&x_position_track, xbeeMsg.x);
                rc_ringbuf_insert(&y_position_track, xbeeMsg.y);
                rc_ringbuf_insert(&z_position_track, xbeeMsg.z);
                
                __diff_function(&x_position_track, &y_position_track, &z_position_track);

                return 0;
            } else 
            {
                // printf("Wrong checksum!");
                // printf("Received %04x ", checksum_received);
                // printf("Function %04x ", checksum_with_function);
                // printf("Calculated %04x\n", checksum_calculated);
                return -1;
            }
        }	
    }

    // Check to see if mocap is still valid (separate from OPEN_LOOP_DESCENT) - just for trackingValid value
    double ms_since_mocap = ((double)rc_nanos_since_epoch() - (double)state_estimate.xbee_time_received_ns) / 1e6;
    if (ms_since_mocap >= settings.mocap_dropout_timeout_ms) 
    {
        xbeeMsg.trackingValid = 0;
    }

    return 0;
}

void __copy_xbee_msg(unsigned char* dataPacket)
{
    switch (settings.xbee_packet_version)
    {
        case 1:
            // printf("Copying XBee Message V1!\n");
            memcpy(&xbeeMsg, dataPacket, opti_data_length);
            break;
        case 2:
            // printf("Copying XBee Message V2!\n");
            memcpy(&xbeeMsg_v2, dataPacket, opti_data_length);
            xbeeMsg.x = xbeeMsg_v2.x;
            xbeeMsg.y = xbeeMsg_v2.y;
            xbeeMsg.z = xbeeMsg_v2.z;
            xbeeMsg.qz = sin(xbeeMsg_v2.yaw/2);
            xbeeMsg.qw = cos(xbeeMsg_v2.yaw/2);
            xbeeMsg.trackingValid = 1;
            break;
        default:
            break;
    }
                    
    return;
}

///////////////////////////////////////
void XBEE_printData()
{
    // Print to terminal
    printf("\r");
    // Time
    if (xbeeMsg.time < 1000000)
        printf("   %u |", xbeeMsg.time);
    else if (xbeeMsg.time < 10000000)
        printf("  %u |", xbeeMsg.time);
    else if (xbeeMsg.time < 100000000)
        printf(" %u |", xbeeMsg.time);
    else
        printf("%u |", xbeeMsg.time);

    // XYZ
    if (xbeeMsg.x < 0)
        printf("%7.6f |", xbeeMsg.x);
    else
        printf(" %7.6f |", xbeeMsg.x);
    if (xbeeMsg.y < 0)
        printf("%7.6f |", xbeeMsg.y);
    else
        printf(" %7.6f |", xbeeMsg.y);
    if (xbeeMsg.z < 0)
        printf("%7.6f |", xbeeMsg.z);
    else
        printf(" %7.6f |", xbeeMsg.z);

    // Quaternion
    if (xbeeMsg.qx < 0)
        printf("%7.6f |", xbeeMsg.qx);
    else
        printf(" %7.6f |", xbeeMsg.qx);
    if (xbeeMsg.qy < 0)
        printf("%7.6f |", xbeeMsg.qy);
    else
        printf(" %7.6f |", xbeeMsg.qy);
    if (xbeeMsg.qz < 0)
        printf("%7.6f |", xbeeMsg.qz);
    else
        printf(" %7.6f |", xbeeMsg.qz);
    if (xbeeMsg.qw < 0)
        printf("%7.6f |", xbeeMsg.qw);
    else
        printf(" %7.6f |", xbeeMsg.qw);

    // Tracking Valid
    printf("   %u   |", xbeeMsg.trackingValid);

    fflush(stdout);
}

void __diff_function(rc_ringbuf_t *x_buffer, rc_ringbuf_t *y_buffer, rc_ringbuf_t *z_buffer) {
    thistime = rc_nanos_since_epoch();
    xbee_dt = (float) (thistime - lastime);
    xbee_dt = xbee_dt / (pow(10,9));

    prev_x_velocity = __diff_function_helper(x_buffer, prev_x_velocity, xbee_dt);
    prev_y_velocity = __diff_function_helper(y_buffer, prev_y_velocity, xbee_dt);
    prev_z_velocity = __diff_function_helper(z_buffer, prev_z_velocity, xbee_dt);

    xbee_x_dot = prev_x_velocity;
    xbee_y_dot = prev_y_velocity;
    xbee_z_dot = prev_z_velocity;
    lastime = thistime;
}

float __diff_function_helper(rc_ringbuf_t *buffer, float prev_velocity, float dt)
{
    float current = rc_ringbuf_get_value(buffer, 0);
    float back1 = rc_ringbuf_get_value(buffer, 1);

    // float answer = 0;
    // for (int i = 0; i < DIFF_POINTS; i++) {
    //     float current = rc_ringbuf_get_value(buffer, i);
    //     answer += current * diff_coeff.d[DIFF_POINTS - 1 - i];
    // }
    // return answer / dt;

    float current_velocity = (-back1 + current)/(dt);
    current_velocity = ((1-ALPHA_CUTOFF)*current_velocity 
            + ALPHA_CUTOFF*(prev_velocity));

    return (current_velocity);
}