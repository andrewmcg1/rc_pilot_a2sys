/**
 * <coordinates.h>
 *
 * @brief   Data structures and functions related to the coordinate systems of the vehicle
 *
 * This modules contains the coordinate system data structures that all other modules should use.
 * Currently, these coordinates are used in the naza_gps.h module and the waypoints.h (path.h)
 * module.
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup Coordinates
 * @{
 */

#ifndef __COORDINATES__
#define __COORDINATES__

#include <math.h>

/**
 * @brief   Lattitude longitude altitude coordinates
 */
typedef struct lla_t
{
    double lat;  //< longitude in degree decimal
    double lon;  //< latitude in degree decimal
    double alt;  //< altitude in m (from GPS)
} lla_t;

/**
 * @brief   Cartesian coordintate waypoint
 *
 * This waypoint is used for localization and desired setpoint (from path).
 * Since the two are related, where the vehicle is at and where it wants to go,
 * this should be standardized over the two modules.
 */
typedef struct waypoint_t
{
    double x, y, z;           ///< position
    double xd, yd, zd;        ///< velocity
    double roll, pitch, yaw;  ///< angles
    double p, q, r;           ///< angular rates
    double t;                 ///< time
    int flag;  ///< flag to specify state transitions, etc. (included to "future proof")

} waypoint_t;

typedef waypoint_t ecef_waypoint_t;
typedef waypoint_t ned_waypoint_t;

/**
 * @brief   Arbitrarily defined origin
 *
 * The program defines an origin for which all local coordinates
 * will be relative to.
 */
typedef struct origin_t
{
    ecef_waypoint_t ecef;
    lla_t lla;
    int initialized;
} origin_t;

extern origin_t origin;

/**
 * @brief   Initialize coordinate system
 *
 * Origin is set to (0,0,0), and considered not initialized.  The origin
 * is considered to be initialized when the first GPS fix is obtained.  That first fix
 * is set as the origin at that time.
 */
int coordinates_init();

/**
 * @brief   Converts lla coordinates to ned (relative to origin)
 *
 * @param[in]   lla     lat,long,alt coordinates to be converted
 *
 * @return  ned waypoint coresponding to the lla coordinates
 */
ned_waypoint_t lla2ned(const lla_t* lla);

/**
 * @brief   Converts lla coordinates to ecef (relative to origin)
 *
 * @param[in]   lla     lat,long,alt coordinates to be converted
 *
 * @return  ecef waypoint coresponding to the lla coordinates
 */
ecef_waypoint_t lla2ecef(const lla_t* lla);

/**
 * @brief   Sets origin to ```origin_lla```
 *
 * @param[in]   origin_lla  lla coordinates to be set as origin
 */
void set_origin(const lla_t* origin_lla);

#endif /*__COORDINATES__*/

/**@} end group Coordinates */