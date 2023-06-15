/**
 * @file naza_gps.c
 * 
 **/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <rc/time.h>
#include <rc/uart.h>


#include <naza_gps.h>
#include <state_estimator.h>

#define BAUDRATE    115200        ///< default gps baudrate
#define TIMEOUT_S   .5

#define GPS_BUS 2

#define GPS_PAYLOAD_LENGTH 58                     ///< size of gps data in bytes
#define COMPASS_PAYLOAD_LENGTH 6                  ///< size of compass data in bytes
#define PAYLOAD_SIZE GPS_PAYLOAD_LENGTH           ///< size of total gps payload

#define GPS_STARTBYTE1 0x55       ///< first payload start byte
#define GPS_STARTBYTE2 0xAA       ///< second payload start byte
#define PAYLOAD_GPS 0x10          ///< gps message ID byte
#define PAYLOAD_COMPASS 0x20      ///< compass message ID byte

/**
 * Structure to store raw gps data as individual segments of data
 */
typedef struct __attribute__ ((packed)) RawGPSData
{
    int32_t dateAndTime;
    int32_t longitude;
    int32_t latitude;
    int32_t altitude;
    int32_t horrizontalAccuracyEstimate;
    int32_t verticalAccuracyEstimate;
    int32_t unknown1;   // unused
    int32_t NEDNorthVelocity;
    int32_t NEDEastVelocity;
    int32_t NEDDownVelocity;
    int16_t positionDOP;
    int16_t verticalDOP;
    int16_t northernDOP;
    int16_t easternDOP;
    uint8_t numberOfSatelites;
    uint8_t unknown2;   // unused
    uint8_t fixType;
    uint8_t unknown3;   // unused
    uint8_t fixStatusFlag;
    int16_t unknown4;   // unused
    uint8_t xorMask;
    int16_t sequenceNumber;
} RawGPSData;

/**
 * Struct to store raw compass data
 */
typedef struct __attribute__ ((packed)) RawCompassData
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} RawCompassData;

/**
 * Struct used to acess timevalues without bitmask
 */
typedef struct __attribute__ ((packed)) Time
{
    uint16_t second  :6;
    uint16_t minute  :6;
    uint16_t hour    :4;
    uint16_t day     :5;
    uint16_t month   :4;
    uint16_t year    :7;
} Time;


gps_data_t gps_data;

RawGPSData gps_data_raw;
RawCompassData compass_data_raw;

/**
 * Functions only to be used locally
 */

/**
 * @brief parses individual raw bytes into buffer
 * 
 * @param current_byte byte to parse
 */
static int __gps_parse(const int current_byte);

/**
 * @brief decodes gps values from buffer
 * 
 * @param messageID type of message in buffer
 */
static void __gps_decode(unsigned char messageID);

/**
 * @brief converts values from raw struct to final format
 */
void __convert_gps_raw_to_final();



int gps_init()
{
    if (rc_uart_init(GPS_BUS, BAUDRATE, TIMEOUT_S, 0, 1, 0))
    {
        printf("Failed to open Serial Port\n");
        return -1;
    }

    // Set gps data to intialilly be invalid
    gps_data.gps_valid = 0;

    return 0;
}

int gps_getData()
{
    unsigned char buffer;
    if(rc_uart_bytes_available(GPS_BUS))
    {

        rc_uart_read_bytes(GPS_BUS, &buffer, 1);
        __gps_parse(buffer);
    }

    if (gps_data.gps_valid)
    {
        if (!origin.initialized)
        {
            set_origin(&gps_data.lla);
            origin.initialized = 1;
        }
        return 0;
    }

    return -1;
}

static void __gps_parse(const int current_byte)
{
    static int count;
    static uint8_t checkSum[2];
    static ParseState parseState = START_BYTE_1;
    static unsigned char messageID;
    static unsigned char messageLength;
    static unsigned char payload[PAYLOAD_SIZE];

    switch (parseState)
    {
    case START_BYTE_1:
        if (current_byte == GPS_STARTBYTE1)
            parseState = START_BYTE_2;
        break;

    case START_BYTE_2:
        if (current_byte == GPS_STARTBYTE2)
        {
            parseState = MESSAGE_ID;
            checkSum[0] = 0;
            checkSum[1] = 0;
        }
        else
            parseState = START_BYTE_1;  // resets state machine
        break;

    case MESSAGE_ID:
        messageID = current_byte;
        checkSum[0] += current_byte;
        checkSum[1] += checkSum[0];
        parseState = VALIDATE_PAYLOAD;
        break;

    case VALIDATE_PAYLOAD:
        if (((messageID == PAYLOAD_GPS) && (current_byte == GPS_PAYLOAD_LENGTH))
           || ((messageID == PAYLOAD_COMPASS) && (current_byte == COMPASS_PAYLOAD_LENGTH)))
        {
            messageLength = current_byte;
            count = 0;
            checkSum[0] += current_byte;
            checkSum[1] += checkSum[0];
            parseState = RECIEVE_PAYLOAD;
        }
        else
            parseState = START_BYTE_1;  // resets state machine
        break;

    case RECIEVE_PAYLOAD:
        payload[count++] = current_byte;
        checkSum[0] += current_byte;
        checkSum[1] += checkSum[0];
        if (count >= messageLength)
        {
            parseState = CHECKSUM_1;
        }
        break;

    case CHECKSUM_1:
        if (current_byte == checkSum[0])
            parseState = CHECKSUM_2;
        else
            parseState = START_BYTE_1;  // resets state machine
        break;

    case CHECKSUM_2:
        if (current_byte == checkSum[1])
            parseState = DECODE_PAYLOAD;
        else
            parseState = START_BYTE_1;  // resets state machine
        break;

    case DECODE_PAYLOAD:
        if (messageID == PAYLOAD_GPS)
        {
            gps_data_raw = *(RawGPSData*)payload;   // casts gps buffer array into raw data struct
        }
        else if (messageID == PAYLOAD_COMPASS)
        {
            compass_data_raw = *(RawCompassData*)payload;   // casts gps buffer array into raw data struct
        }
        parseState = START_BYTE_1; // resets state machine

        __gps_decode(messageID);
        __convert_gps_raw_to_final();
    }
}

static void __gps_decode(unsigned char messageID)
{
    uint8_t mask = 0;

    if (messageID == PAYLOAD_GPS)
    {
        mask = gps_data_raw.xorMask;    // xor mask comes from gps

        uint8_t* temp = (uint8_t*)&gps_data_raw;    // cast struct to array

        for(int i = 0; i < GPS_PAYLOAD_LENGTH; i++)
            temp[i] ^= mask;    // mask each byte of array

        gps_data_raw = *(RawGPSData*)temp;

        gps_data_raw.numberOfSatelites ^= mask; // unmask
        gps_data_raw.unknown2 ^= mask;  // unmask
        gps_data_raw.sequenceNumber ^= (mask << 8) | mask;  // unmask

    }
    else if (messageID == PAYLOAD_COMPASS)
    {
        mask = compass_data_raw.z & 0x00ff;
		mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));  // mask from forum post

        uint8_t* temp = (uint8_t*)&compass_data_raw;    // cast struct to array

        for(int i = 0; i < COMPASS_PAYLOAD_LENGTH; i++)
            temp[i] ^= mask;    // mask each byte of array
        
        compass_data_raw = *(RawCompassData*)temp;  // cast array back to struct

        compass_data_raw.z ^= mask; // unmask first byte of z
    }
    return;
}

void __convert_gps_raw_to_final()
{   
    Time time;
    memcpy(&time, &gps_data_raw.dateAndTime, 4);    // copies dateAndTime bytes into time struct

    /**
     * sets final format struct
     */
    gps_data.year = time.year;  
    gps_data.month = time.month;
    gps_data.day = time.day;
    gps_data.hour = time.hour;
    gps_data.minute = time.minute;
    gps_data.second = time.second;

    gps_data.lla.lat = (double)gps_data_raw.latitude / 10000000.0;  // converts units
    gps_data.lla.lon = (double)gps_data_raw.longitude / 10000000.0;  // converts units
    gps_data.lla.alt = (double)gps_data_raw.altitude / 1000.0;  // converts units


    double nVel = (double)gps_data_raw.NEDNorthVelocity / 100.0;
    double eVel = (double)gps_data_raw.NEDEastVelocity / 100.0;
    gps_data.spd = sqrt(nVel * nVel + eVel * eVel); // gets magnitude of velocity


    gps_data.cog = atan2(eVel, nVel);
    if (gps_data.cog < 0) 
        gps_data.cog += 360.0;


    gps_data.gpsVsi = -(double)gps_data_raw.NEDDownVelocity / 100.0;  // converts units

    gps_data.vdop = (double)gps_data_raw.verticalDOP / 100.0;  // converts units

    double ndop = (double)gps_data_raw.northernDOP / 100.0;
    double edop = (double)gps_data_raw.easternDOP / 100.0;
    gps_data.hdop = sqrt(ndop * ndop + edop * edop);    // gets magnitude of dilution of precision

    gps_data.sat = gps_data_raw.numberOfSatelites;


    if(gps_data_raw.fixStatusFlag == 0x02 && gps_data_raw.fixType != NO_FIX)
        gps_data.fix = FIX_DGPS;
    else
        gps_data.fix = gps_data_raw.fixType;


    // Convert current location to north east down coordinates;
    gps_data.ned = lla2ned(&gps_data.lla);

    // Log time that data is received
    gps_data.gps_data_received_ns = rc_nanos_since_epoch();

    // Check if gps is valid (> 4 satelites for 3D)
    gps_data.gps_valid = (gps_data.sat >= 4);



    // Continuous calibration of compass data
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (compass_data_raw.x > magXMax) 
        magXMax = compass_data_raw.x;
    if (compass_data_raw.x < magXMin) 
        magXMin = compass_data_raw.x;
    if (compass_data_raw.y > magYMax) 
        magYMax = compass_data_raw.y;
    if (compass_data_raw.y < magYMin) 
        magYMin = compass_data_raw.y;
    gps_data.headingNc = -atan2(compass_data_raw.y - ((magYMax + magYMin) / 2), 
                                compass_data_raw.x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
    if (gps_data.headingNc < 0)
        gps_data.headingNc += 360.0;

}

