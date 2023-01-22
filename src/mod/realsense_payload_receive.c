/*
 * @file realsense_payload_receive.c
 *
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
#include <realsense_payload_receive.h>
#include <crc16.h>

landing_command_t rspLandingCommandMsg; // Defined as extern in realsense_payload_receive.h
pose_update_t rspPoseUpdateMsg;         // Defined as extern in realsense_payload_receive.h
int realsense_portID;                   // Defined as extern in realsense_payload_receive.h

int REALSENSE_init(const char *realsense_port)
{
    int baudRate = 57600;
    realsense_portID = serial_open(realsense_port, baudRate, 0);  // Nonblocking = 0, BLOCKING = 1
    if (realsense_portID == -1)
    {
        printf("Failed to open Serial Port %s\n",realsense_port);
        return -1;
    }
    return 0;
}

// Read messages received from realsense payload
int REALSENSE_getData()
{
    // Init Packet Pointers
    static unsigned char serialPacket[RSP_MAX_PACKET_LENGTH];
    static unsigned char* dataPacket = serialPacket + RSP_NUM_FRAMING_BYTES;
    static unsigned char* ptr = serialPacket;

    // Init Other relevant data
    static int currDataLength = 0;
    static int currPacketLength = 0;
    static int currMsgID = 0;

    while(read(realsense_portID,ptr,1) > 0) {    
        // 1) if the first Byte is wrong keep looking
        if( (ptr == serialPacket) && (ptr[0] != RSP_START_BYTE1) ) {
            // std::cout << "First Byte Wrong!" << std::endl;
            ptr = serialPacket;
            continue;
        }

        // 2) if the second Byte is wrong keep looking
        if( (ptr == serialPacket + 1) && (ptr[0] != RSP_START_BYTE2) ) {
            // std::cout << "Second Byte Wrong!" << std::endl;
            ptr = serialPacket;
            continue;
        }

        // 3) Determine message data length with the third byte
        if (ptr == serialPacket + 2) {
            currDataLength = ptr[0];
            currPacketLength = currDataLength + RSP_NUM_FRAMING_BYTES + RSP_NUM_CHECKSUM_BYTES;
        }

        // 4) Determine Message type with the fourth byte
        if (ptr == serialPacket + 3) {
            currMsgID = ptr[0];
        }

        // 5) Keep reading in more bytes
        ptr++;	

        // 6) Once we have all of the Bytes, Process them!
        if( (ptr-serialPacket) == currPacketLength) {
            // std::cout << "Valid Packet!" << std::endl;
            ptr = dataPacket;

            uint16_t checksum_calculated = fletcher16(ptr, currDataLength);
            uint16_t checksum_received = 0;
            memcpy(&checksum_received, &ptr[currDataLength], sizeof(checksum_received));
            // printf("Checksum calculated: %u; Checksum Recieved: %u \n", checksum_calculated, checksum_received);
            // If checksums don't match set message ID to 0; indicating bad message
            if (checksum_received != checksum_calculated)
            {
                currMsgID = 0;
                // printf("Checksum failed\n");
            }

            // std::cout << "Msg ID = " << currMsgID << std::endl;
            switch (currMsgID) {
                case 1:
                    memcpy(&rspPoseUpdateMsg, ptr, currDataLength);
                    break;
                case 2:
                    memcpy(&rspLandingCommandMsg, ptr, currDataLength);
                    break;
                default:
                    // std::cout << "Invalid Msg ID!" << std::endl;
                    ptr = serialPacket;
                    return 0;
            }

            ptr = serialPacket;  
            return currMsgID;
        }	
    }
    return 0;
}
