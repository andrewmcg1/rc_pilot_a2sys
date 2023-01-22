/**
 * <waypoints.h>
 *
 * @brief   Functions to read the waypoint file and handle the path
 *
 * Program expects waypoint file of the form (no line numbers in actual file):
 *
 * ~~~
 * 1:    t x y z xd yd zd r p y rd pd yd
 * 2:    t x y z xd yd zd r p y rd pd yd
 *         ...
 * n:    t x y z xd yd zd r p y rd pd yd
 * ~~~
 *
 * where line i contains waypoint i with position, velocity and time respectively.
 * Memory will be dynamically allocated for the path and the path will be stored
 * as a set of waypoints defined by the file
 *
 * TODO: Possibly rename (path maybe?), nameing conflicts with the
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup Waypoints
 * @{
 */

#ifndef __PATH__
#define __PATH__

#include <stddef.h>

#include <coordinates.h>
#include <realsense_payload_receive.h>
#include <state_estimator.h>
#include <math_utils.h>

/**
 * @brief       Read waypoint file and initialize path
 *
 * Checks for a valid file and counts the number of waypoints specified.  Based
 * on the number of waypoints, memory for the path is dynamically allocated (freed in cleanup).
 * Waypoints are then sequentially read into the path.  If any invalid waypoints are specified,
 * i.e. not in the form <x y z xd yd zd t> (all floats), then the intialization fails and the
 * path stays unitialized.
 *
 * @param[in]   file_path   string containing the relative path to the waypoint file
 *
 * @return      0 on success, -1 on failure
 */
int path_load_from_file(const char* file_path);

/**
 * @brief       Frees memory allocated in path and "unitializes" path variable
 */
void path_cleanup();

/**
 * @brief       Plan a path based on realsense payload landing command
 * 
 * Very simple version with no-splines. Allows incremental testing.
 * 
 * @return      0 on success, -1 on failure
 */
int path_plan_from_rsp_cmd_3pts();

/**
 * @brief       Plan a path based on realsense payload landing command
 * 
 * @return      0 on success, -1 on failure
 */
int path_plan_from_rsp_cmd();


typedef struct quintic_spline_1d_t
{
    float c0, c1, c2, c3, c4, c5;
} quintic_spline_1d_t;

/**
 * @brief       Compute quintic spline coefficients for simple 1d path.
 * 
 * Starting and ending velocity and acceleration are 0.
 * 
 * @return      quintic_spline_1d_t with proper coefficients
 */
quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt);

/**
 * @brief       Compute position along spline based on time
 * 
 * @return      1d position along spline
 */
float compute_spline_position(quintic_spline_1d_t* the_spline, float t);

typedef struct path_t
{
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    size_t len;             ///< length of the path (number of waypoints)

    int initialized;  ///< 1 if initialized, 0 if uninitialized
} path_t;

/**
 * @brief       Initial values for path_t
 */
#define PATH_INITIALIZER                              \
    {                                                 \
        .waypoints = NULL, .len = 0, .initialized = 0 \
    }

extern path_t path;

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1

#endif /*__PATH__ */

/* @} Waypoints */