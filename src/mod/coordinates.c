/**
 * @file coordinates.c
 *
 **/

#include <coordinates.h>

origin_t origin;
static const double radius_e = 6368502.0;  // msl radius of earth in ann arbor

int coordinates_init()
{
    lla_t origin_lla = {0, 0, 0};
    origin.lla = origin_lla;
    origin.ecef = lla2ecef(&origin.lla);

    // Set origin as uninitialized until good gps data is received
    origin.initialized = 0;

    return 0;
}

ned_waypoint_t lla2ned(const lla_t* lla)
{
    ecef_waypoint_t p = lla2ecef(lla);
    p.x -= origin.ecef.x;
    p.y -= origin.ecef.y;
    p.z -= origin.ecef.z;

    const double theta = (-origin.lla.lat - 90) * M_PI / 180;
    const double phi = origin.lla.lon * M_PI / 180;

    ned_waypoint_t pned;
    pned.x = (p.x * cos(phi) + p.y * sin(phi)) * cos(theta) - p.z * sin(theta);
    pned.y = -p.x * sin(phi) + p.y * cos(phi);
    pned.z = (p.x * cos(phi) + p.y * sin(phi)) * sin(theta) + p.z * cos(theta);

    return pned;
}

ecef_waypoint_t lla2ecef(const lla_t* lla)
{
    ecef_waypoint_t p;
    const double re = radius_e + lla->alt;
    const double phi = lla->lon * M_PI / 180;
    const double theta = lla->lat * M_PI / 180;

    p.x = re * cos(theta) * cos(phi);
    p.y = re * cos(theta) * sin(phi);
    p.z = re * sin(theta);

    return p;
}

void set_origin(const lla_t* origin_lla)
{
    origin.lla = *origin_lla;
    origin.ecef = lla2ecef(origin_lla);
}