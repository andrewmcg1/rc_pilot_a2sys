/**
 * <naza_gps.h>
 *
 * @brief Data structurs and functions to obtain gps readings from DJI Naza V2 Gps
 *
 * Development for this module is based on <a
 * href="https://www.rcgroups.com/forums/showthread.php?1995704-DJI-NAZA-GPS-communication-protocol-NazaDecoder-Arduino-library"
 * target="_blank">this forum post.</a>
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup NazaGps
 * @{
 */

#ifndef __NAZA_GPS__
#define __NAZA_GPS__

#include <stdint.h>

#include <coordinates.h>

/**
 * @brief   Gps fix type
 */
typedef enum
{
    NO_FIX = 0,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGPS = 4
} fixType_t;

/**
 *  @brief      Naza gps data structure
 */
typedef struct gps_data_t
{
    lla_t lla;
    ned_waypoint_t ned;
    double spd;        ///< speed in m/s
    fixType_t fix;     ///< fix type
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

/**
 * @brief   Set up serial connection to the naza gps unit
 *
 * The serial port is defined in the code to be the GPS (uart2) port on the beaglebone
 *
 * @return  0 on success, -1 on failure (not good at detecting failure)
 */
int gps_init();

/**
 * @brief   attempt to extract data from naza gps
 *
 * Function attempts to read one byte at a time from the naza gps unit
 * until all bytes are read.  The data is parsed one byte at a time.
 *
 * @return  0 on success, -1 on failure
 */
int gps_getData();

#endif /* __NAZA_GPS__ */

/* @} end group NazaGps */