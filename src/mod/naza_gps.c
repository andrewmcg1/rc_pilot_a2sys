/**
 * @file naza_gps.c
 **/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rc/time.h>

#include <naza_gps.h>
#include <serial_com.h>
#include <state_estimator.h>

#define PAYLOAD_SIZE 64
#define GPS_STARTBYTE1 0x55       ///< first start byte
#define GPS_STARTBYTE2 0xAA       ///< second start byte
#define PAYLOAD_GPS 0x10          ///< gps message byte
#define PAYLOAD_GPS_LEN 0x3A      ///< gps message length
#define PAYLOAD_COMPASS 0x20      ///< compass message byte
#define PAYLOAD_COMPASS_LEN 0x06  ///< compass message length

unsigned char payload[PAYLOAD_SIZE];  // TODO: static? for each of these variables...
unsigned char cs1, cs2;               ///< checksum 1 and 2
int msgId, msgLen;
int gps_portID;

gps_data_t gps_data;

/**
 * Functions only to be used locally
 */
static int __gps_parse(const int input);
static void __updateCS(const int input);
static void __gps_decode();
static int32_t __decodeLong(uint8_t idx, uint8_t mask);
static int16_t __decodeShort(uint8_t idx, uint8_t mask);

int gps_init()
{
    int baudRate = 115200;
    gps_portID = serial_open("/dev/ttyS2", baudRate, 0);  // TODO: Add serial port to settings file
    if (gps_portID == -1)
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

    while (read(gps_portID, &buffer, 1) > 0)
    {
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
    static int seq = 0, cnt = 0;

    if (seq == 0 && input == GPS_STARTBYTE1)
    {
        ++seq;
    }
    else if (seq == 1 && input == GPS_STARTBYTE2)
    {
        cs1 = 0;
        cs2 = 0;
        ++seq;
    }
    else if (seq == 2)
    {
        msgId = input;
        __updateCS(input);
        ++seq;
    }
    else if (seq == 3 && ((msgId == PAYLOAD_GPS && input == PAYLOAD_GPS_LEN) ||
                             (msgId == PAYLOAD_COMPASS && input == PAYLOAD_COMPASS_LEN)))
    {
        msgLen = input;
        cnt = 0;
        __updateCS(input);
        ++seq;
    }
    else if (seq == 4)
    {
        payload[cnt++] = input;
        __updateCS(input);
        if (cnt >= msgLen)
        {
            ++seq;
        }
    }
    else if (seq == 5 && input == cs1)
    {
        ++seq;
    }
    else if (seq == 6 && input == cs2)
    {
        ++seq;
    }
    else
    {
        seq = 0;
        return -1;
    }

    // Full message read into buffer, both checksums good
    if (seq == 7)
    {
        seq = 0;
        __gps_decode();
    }
    return 0;
}

static void __updateCS(const int input)
{
    cs1 += input;
    cs2 += cs1;
}

static void __gps_decode()
{
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (msgId == PAYLOAD_GPS)
    {
        uint8_t mask = payload[55];
        uint32_t time = __decodeLong(0, mask);
        gps_data.second = time & 0b00111111;
        time >>= 6;
        gps_data.minute = time & 0b00111111;
        time >>= 6;
        gps_data.hour = time & 0b00001111;
        time >>= 4;
        gps_data.day = time & 0b00011111;
        time >>= 5;
        if (gps_data.hour > 7) gps_data.day++;
        gps_data.month = time & 0b00001111;
        time >>= 4;
        gps_data.year = time & 0b01111111;

        gps_data.lla.lon = (double)__decodeLong(4, mask) / 10000000;
        gps_data.lla.lat = (double)__decodeLong(8, mask) / 10000000;
        gps_data.lla.alt = (double)__decodeLong(12, mask) / 1000;

        double nVel = (double)__decodeLong(28, mask) / 100;
        double eVel = (double)__decodeLong(32, mask) / 100;
        gps_data.spd = sqrt(nVel * nVel + eVel * eVel);
        gps_data.cog = atan2(eVel, nVel) * 180.0 / M_PI;
        if (gps_data.cog < 0) gps_data.cog += 360.0;
        gps_data.gpsVsi = -(double)__decodeLong(36, mask) / 100;
        gps_data.vdop = (double)__decodeShort(42, mask) / 100;
        double ndop = (double)__decodeShort(44, mask) / 100;
        double edop = (double)__decodeShort(46, mask) / 100;
        gps_data.hdop = sqrt(ndop * ndop + edop * edop);
        gps_data.sat = payload[48];
        uint8_t fixType = payload[50] ^ mask;
        uint8_t fixFlags = payload[52] ^ mask;
        switch (fixType)
        {
            case 2:
                gps_data.fix = FIX_2D;
                break;
            case 3:
                gps_data.fix = FIX_3D;
                break;
            default:
                gps_data.fix = NO_FIX;
                break;
        }
        if ((gps_data.fix != NO_FIX) && (fixFlags & 0x02))
        {
            gps_data.fix = FIX_DGPS;
        }

        // Convert current location to north east down coordinates;
        gps_data.ned = lla2ned(&gps_data.lla);

        // Log time that data is received
        gps_data.gps_data_received_ns = rc_nanos_since_epoch();

        // Check if gps is valid (> 4 satelites for 3D)
        gps_data.gps_valid = (gps_data.sat >= 4);
    }
    else if (msgId == PAYLOAD_COMPASS)
    {
        uint8_t mask = payload[4];
        mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^
               (((mask & 0x01) << 3) | ((mask & 0x01) << 7));
        int16_t x = __decodeShort(0, mask);
        int16_t y = __decodeShort(2, mask);
        if (x > magXMax) magXMax = x;
        if (x < magXMin) magXMin = x;
        if (y > magYMax) magYMax = y;
        if (y < magYMin) magYMin = y;
        gps_data.headingNc =
            -atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
        if (gps_data.headingNc < 0)
        {
            gps_data.headingNc += 360.0;
        }
    }
    return;
}

static int32_t __decodeLong(uint8_t idx, uint8_t mask)
{
    union {
        uint32_t l;
        uint8_t b[4];
    } val;

    for (int i = 0; i < 4; i++)
    {
        val.b[i] = payload[idx + i] ^ mask;
    }

    return val.l;
}

static int16_t __decodeShort(uint8_t idx, uint8_t mask)
{
    union {
        uint16_t s;
        uint8_t b[2];
    } val;

    for (int i = 0; i < 2; i++)
    {
        val.b[i] = payload[idx + i] ^ mask;
    }

    return val.s;
}