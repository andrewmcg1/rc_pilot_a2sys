/**
 * @file naza_gps.c
 **/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rc/time.h>
#include <rc/uart.h>


#include <naza_gps.h>
#include <state_estimator.h>

#define BAUDRATE    115200
#define TIMEOUT_S   .5

#define GPS_BUS 2

#define PAYLOAD_SIZE 64
#define GPS_PAYLOAD_LENGTH 58
#define COMPASS_PAYLOAD_LENGTH 6

#define GPS_STARTBYTE1 0x55       ///< first start byte
#define GPS_STARTBYTE2 0xAA       ///< second start byte
#define PAYLOAD_GPS 0x10          ///< gps message byte
#define PAYLOAD_COMPASS 0x20      ///< compass message byte

/**
 * Structure to store raw gps data as individual segments of data
 */
typedef struct RawGPSData
{
    long int dateAndTime;
    long int longitude;
    long int latitude;
    long int altitude;
    long int horrizontalAccuracyEstimate;
    long int verticalAccuracyEstimate;
    long int unknown1;
    long int NEDNorthVelocity;
    long int NEDEashVelocity;
    long int NEDDownVelocity;
    short int positionDOP;
    short int verticalDOP;
    short int northernDOP;
    short int easternDOP;
    unsigned char numberOfSatelites;
    unsigned char unknown2;
    unsigned char fixType;
    unsigned char unknown3;
    unsigned char fixStatusFlag;
    short int unknown4;
    unsigned char xorMask;
    short int sequenceNumber;
} RawGPSData;

/**
 * Struct to store raw compass data
 */
typedef struct RawCompassData
{
    short int x;
    short int y;
    short int z;
} RawCompassData;

/**
 * This union is used to quickly convert between raw data structs and uint8_t arrays
 */
typedef union struct_to_int
{
    RawGPSData gps_structure;
    RawCompassData compass_structure;
    uint8_t struct_as_int[PAYLOAD_SIZE];
} struct_to_int;

gps_data_t gps_data;

RawGPSData gps_data_raw;
RawCompassData compass_data_raw;

/**
 * Functions only to be used locally
 */
static int __gps_parse(const int input);
static void __updateCS(const int input);
static void __gps_decode(unsigned char messageID);

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

static int __gps_parse(const int input)
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
        if (input == GPS_STARTBYTE1)
            parseState = START_BYTE_2;
        break;

    case START_BYTE_2:
        if (input == GPS_STARTBYTE2)
        {
            parseState = MESSAGE_ID;
            checkSum[0] = 0;
            checkSum[1] = 0;
        }
        else
            parseState = START_BYTE_1;
        break;

    case MESSAGE_ID:
        messageID = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        parseState = VALIDATE_PAYLOAD;
        break;

    case VALIDATE_PAYLOAD:
        if ((messageID == PAYLOAD_GPS && input == GPS_PAYLOAD_LENGTH)
           || messageID == PAYLOAD_COMPASS && input == COMPASS_PAYLOAD_LENGTH)
        {
            messageLength = input;
            count = 0;
            checkSum[0] += input;
            checkSum[1] += checkSum[0];
            parseState = RECIEVE_PAYLOAD;
        }
        else
            parseState = START_BYTE_1;
        break;

    case RECIEVE_PAYLOAD:
        payload[count++] = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        if (count >= messageLength)
        {
            parseState = CHECKSUM_1;
        }
        break;

    case CHECKSUM_1:
        if (input == checkSum[0])
            parseState = CHECKSUM_2;
        else
            parseState = START_BYTE_1;
        break;

    case CHECKSUM_2:
        if (input == checkSum[1])
            parseState = DECODE_PAYLOAD;
        else
            parseState = START_BYTE_1;
        break;

    case DECODE_PAYLOAD:
        if (messageID == PAYLOAD_GPS)
        {
            gps_data_raw = *(RawGPSData*)payload;
        }
        else if (messageID == PAYLOAD_COMPASS)
        {
            compass_data_raw = *(RawCompassData*)payload;
        }
        parseState = START_BYTE_1;
        __gps_decode(messageID);
    }

    return 0;
}

static void __gps_decode(unsigned char messageID)
{
    struct_to_int gps_union;
    static int16_t magXMax, magXMin, magYMax, magYMin;
    uint8_t mask = 0;
    uint8_t mask_bits[8];

    if (messageID == PAYLOAD_GPS)
    {
        mask = gps_data_raw.xorMask;

        gps_union.gps_structure = gps_data_raw;

        for(int i = 0; i < GPS_PAYLOAD_LENGTH; i++)
            gps_union.struct_as_int[i] ^= mask;

        gps_data_raw = gps_union.gps_structure;

        gps_data_raw.numberOfSatelites ^= mask;
        gps_data_raw.unknown2 ^= mask;
        gps_data_raw.sequenceNumber ^= (mask << 8) & mask;

    }
    else if (messageID == PAYLOAD_COMPASS)
    {
        mask = compass_data_raw.z_vector & 0x00ff;
		mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));

        gps_union.compass_structure = compass_data_raw;

        for(int i = 0; i < GPS_PAYLOAD_LENGTH; i++)
            gps_union.struct_as_int[i] ^= mask;

        compass_data_raw = gps_union.compass_structure;

        compass_data_raw.z_vector ^= mask;
    }
    return;
}

int convert_gps_raw_to_final()
{
    uint32_t time = gps_data_raw.dateAndTime;

    gps_data.second = time & 0b00111111;
    time >>= 6;
    gps_data.minute = time & 0b00111111;
    time >>= 6;
    gps_data.hour = time & 0b00001111;
    time >>= 4;
    gps_data.day = time & 0b00011111;
    time >>= 5;
    if (gps_data.hour > 7) 
        gps_data.day++;
    gps_data.month = time & 0b00001111;
    time >>= 4;
    gps_data.year = time & 0b01111111;


    gps_data.lla.lat = (double)gps_data_raw.latitude / 10000000.0;
    gps_data.lla.lon = (double)gps_data_raw.longitude / 10000000.0;
    gps_data.lla.alt = (double)gps_data_raw.altitude / 1000.0;


    double Nvel = (double)gps_data_raw.NEDNorthVelocity / 100.0;
    double Evel = (double)gps_data_raw.NEDEastVelocity / 100.0;
    gps_data.spd = sqrt(nVel * nVel + eVel * eVel);


    gps_data.cog = atan2(eVel, nVel);
    if (gps_data.cog < 0) 
        gps_data.cog += 360.0;


    gps_data.gpsVsi = -(double)gps_data_raw.NEDDownVelocity / 100.0;

    gps_data.vdop = (double)gps_data_raw.verticalDOP / 100.0;

    double ndop = (double)gps_data_raw.northernDOP / 100.0;
    double edop = (double)gps_data_raw.easternDOP / 100.0;
    gps_data.hdop = sqrt(ndop * ndop + edop * edop);

    gps_data.sat = dps_data_raw.numberOfSatelites;


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



    // Compass data
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (compass_data_raw.x > magXMax) 
        magXMax = x;
    if (compass_data_raw.x < magXMin) 
        magXMin = x;
    if (compass_data_raw.y > magYMax) 
        magYMax = y;
    if (compass_data_raw.y < magYMin) 
        magYMin = y;
    gps_data.headingNc = -atan2(compass_data_raw.y - ((magYMax + magYMin) / 2), 
                                compass_data_raw.x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
    if (gps_data.headingNc < 0)
        gps_data.headingNc += 360.0;

}

