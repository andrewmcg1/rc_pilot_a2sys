#ifndef __NAZA_GPS__
#define __NAZA_GPS__

#include <stdint.h>

enum _FixType
{
    NO_FIX = 0,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGPS = 4,
};
typedef enum _FixType FixType;

enum _ParseState
{
    START_BYTE_1,
    START_BYTE_2,
    MESSAGE_ID,
    VALIDATE_PAYLOAD,
    RECIEVE_PAYLOAD,
    CHECKSUM_1,
    CHECKSUM_2,
    DECODE_PAYLOAD
};
typedef enum _ParseState ParseState;

typedef struct gps_data_t
{
    lla_t lla;
    ned_waypoint_t ned;
    double spd;        ///< speed in m/s
    FixType fix;     ///< fix type
    uint8_t sat;       ///< number of satellites
    double headingNc;  ///< heading (not tilt compensated) in degrees
    double cog;        ///< course over ground
    double gpsVsi;     ///< vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double hdop;       ///< horizontal dilution of precision
    double vdop;       ///< vertical dilution of precision
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint64_t gps_data_received_ns;  ///< time on beaglebond board that gps is received
    uint8_t gps_valid;

} gps_data_t;

extern gps_data_t gps_data;

int gps_init();
int gps_getData();

#endif
