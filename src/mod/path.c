/**
 * @file waypoint.c
 **/

#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()

#include <path.h>

path_t path = PATH_INITIALIZER;

/*********************************
 * Functions for internal use only
 *
 */

/**
 * @brief   Count the number of lines in a file, indicates number of waypoints
 *
 * @return  Number of lines in the file
 */
static int __count_file_lines(const char* file_path);

/**
 * @brief   Read all of the waypoints from a file into the path variable
 *
 * @return  0 on success, -1 on failure
 */
static int __read_waypoints(FILE* fd);
/**
 * ********************************
 */

int path_load_from_file(const char* file_path)
{
    // Clear any previously stored path, set init to 0
    path_cleanup();

    // Check for valid file
    if (access(file_path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: waypoint file missing\n");
        return -1;
    }

    // Count number of waypoints contained in file
    path.len = __count_file_lines(file_path);

    // Open file for waypoint reading
    FILE* fd = fopen(file_path, "r");

    // Read path size and allocate waypoint memory
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
    if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Read waypoints from file
    if (__read_waypoints(fd) < 0)
    {
        path_cleanup(); //Added to prevent potential memory leak
        fprintf(stderr, "ERROR: failed reading waypoint file\n");
        return -1;
    }

    fclose(fd);

    path.initialized = 1;
    return 0;
}

void path_cleanup()
{
    free(path.waypoints);
    path.waypoints = NULL;
    path.len = 0;
    path.initialized = 0;
}

static int __count_file_lines(const char* file_path)
{
    FILE* fd = fopen(file_path, "r");
    int c = 0;
    size_t count = 0;

    c = getc(fd);
    while (c != EOF)
    {
        if (c == '\n' || c == EOF)
        {
            ++count;
        }
        c = getc(fd);
    }

    fclose(fd);
    return count;
}

static int __read_waypoints(FILE* fd)
{
    int rcount = 0;
    int waypoint_num = 0;

    while (rcount != EOF)
    {
        // Read formated file line (13 doubles and 1 int)
        rcount = fscanf(fd, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %i ",
            &path.waypoints[waypoint_num].t, &path.waypoints[waypoint_num].x,
            &path.waypoints[waypoint_num].y, &path.waypoints[waypoint_num].z,
            &path.waypoints[waypoint_num].xd, &path.waypoints[waypoint_num].yd,
            &path.waypoints[waypoint_num].zd, &path.waypoints[waypoint_num].roll,
            &path.waypoints[waypoint_num].pitch, &path.waypoints[waypoint_num].yaw,
            &path.waypoints[waypoint_num].p, &path.waypoints[waypoint_num].q,
            &path.waypoints[waypoint_num].r, &path.waypoints[waypoint_num].flag);

        // If not end of file, but an invalid read (waypoints have 14 values)
        if (rcount != EOF && rcount != 14)
        {
            fprintf(stderr, "ERROR: invalid waypoint read from line: %i\n",
                waypoint_num + 1);  // lines 1 indexed, waypoints zero indexed
            return -1;
        }

        // Increment line number for next iteration
        ++waypoint_num;
    }
    return 0;
}


quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt)
{
    quintic_spline_1d_t simple_quintic_spline;

    simple_quintic_spline.c0 = 0;
    simple_quintic_spline.c1 = 0;
    simple_quintic_spline.c2 = 0;
    simple_quintic_spline.c3 =  10 * dx / pow(dt,3);
    simple_quintic_spline.c4 = -15 * dx / pow(dt,4);
    simple_quintic_spline.c5 =   6 * dx / pow(dt,5);

    return simple_quintic_spline;
}

float compute_spline_position(quintic_spline_1d_t* the_spline, float t)
{
    return the_spline->c0 
         + the_spline->c3 * pow(t,3)
         + the_spline->c4 * pow(t,4)
         + the_spline->c5 * pow(t,5);
}


int path_plan_from_rsp_cmd_3pts()
{
    // Clear any previously stored path, set init to 0
    path_cleanup();

    // Simple 3-waypoint path
    path.len = 3;
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Pt - 1 (current location)
    path.waypoints[0].t = 1;
    path.waypoints[0].x = state_estimate.X;
    path.waypoints[0].y = state_estimate.Y;
    path.waypoints[0].z = state_estimate.Z;
    path.waypoints[0].xd = 0;
    path.waypoints[0].yd = 0;
    path.waypoints[0].zd = 0;
    path.waypoints[0].roll = 0;
    path.waypoints[0].pitch = 0;
    path.waypoints[0].yaw = 0;
    path.waypoints[0].p = 0;
    path.waypoints[0].q = 0;
    path.waypoints[0].r = 0;
    path.waypoints[0].flag = 0;

    // Pt - 2 (above landing spot)
    path.waypoints[1].t = 6;
    path.waypoints[1].x = rspLandingCommandMsg.x;
    path.waypoints[1].y = rspLandingCommandMsg.y;
    path.waypoints[1].z = state_estimate.Z;
    path.waypoints[1].xd = 0;
    path.waypoints[1].yd = 0;
    path.waypoints[1].zd = 0;
    path.waypoints[1].roll = 0;
    path.waypoints[1].pitch = 0;
    path.waypoints[1].yaw = 0;
    path.waypoints[1].p = 0;
    path.waypoints[1].q = 0;
    path.waypoints[1].r = 0;
    path.waypoints[1].flag = 0;

    // Pt - 3 (at landing spot)
    path.waypoints[2].t = 11;
    path.waypoints[2].x = rspLandingCommandMsg.x;
    path.waypoints[2].y = rspLandingCommandMsg.y;
    path.waypoints[2].z = 0;
    path.waypoints[2].xd = 0;
    path.waypoints[2].yd = 0;
    path.waypoints[2].zd = 0;
    path.waypoints[2].roll = 0;
    path.waypoints[2].pitch = 0;
    path.waypoints[2].yaw = 0;
    path.waypoints[2].p = 0;
    path.waypoints[2].q = 0;
    path.waypoints[2].r = 0;
    path.waypoints[2].flag = 0;

    path.initialized = 1;
    return 0;
}

int path_plan_from_rsp_cmd()
{
    // x1 is our current position
    rc_vector_t x1  = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x1, 3);
    x1.d[0] = state_estimate.X;
    x1.d[1] = state_estimate.Y;
    x1.d[2] = state_estimate.Z;
    double t1 = 0.0;

    // x2 is our current z but directly above the landing point
    rc_vector_t x2 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x2, 3);
    x2.d[0] = rspLandingCommandMsg.x;
    x2.d[1] = rspLandingCommandMsg.y;
    x2.d[2] = state_estimate.Z;
    double t2 = 5.0;

    // x3 is the landing point with z=0 so no funny business occurs
    rc_vector_t x3  = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x3, 3);
    x3.d[0] = rspLandingCommandMsg.x;
    x3.d[1] = rspLandingCommandMsg.y;
    x3.d[2] = 0;
    double t3 = 10.0;

    // 
    int num_pts_per_spline = 500; // 5 sec, 500 pts, 100 pts/sec = 100Hz
    int num_pts_total = num_pts_per_spline * 2; // 2 segments

    path_cleanup(); // Clear any previously stored path, set init to 0

    path.len = num_pts_total;
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Start at t=0
    double t_curr = 0;
    double s_curr = 0;
    double x_curr = 0;
    double y_curr = 0;
    double z_curr = 0;

    // Setup Segment #1
    double dx = x2.d[0] - x1.d[0];
    double dy = x2.d[1] - x1.d[1];
    double dz = x2.d[2] - x1.d[2];
    double d_len = sqrt(dx*dx + dy*dy + dz*dz);
    quintic_spline_1d_t q_spline_1  = make_1d_quintic_spline(d_len, t2 - t1);

    // Run through Segment #1
    for (int i=0; i < num_pts_per_spline; i++) 
    {
        // 1) Get 1d position
        t_curr = ( ((double) i) / ((double) (num_pts_per_spline-1))) * (t2 - t1);
        s_curr = compute_spline_position(&q_spline_1, t_curr);

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            x_curr = (s_curr / d_len) * dx  + x1.d[0];
            y_curr = (s_curr / d_len) * dy  + x1.d[1];
            z_curr = (s_curr / d_len) * dz  + x1.d[2];
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            x_curr = x1.d[0];
            y_curr = x1.d[1];
            z_curr = x1.d[2];
        }
        

        // 3) Write to the path
        path.waypoints[i].t = t_curr + t1;
        path.waypoints[i].x = x_curr;
        path.waypoints[i].y = y_curr;
        path.waypoints[i].z = z_curr;
        path.waypoints[i].xd = 0;
        path.waypoints[i].yd = 0;
        path.waypoints[i].zd = 0;
        path.waypoints[i].roll = 0;
        path.waypoints[i].pitch = 0;
        path.waypoints[i].yaw = state_estimate.continuous_yaw;
        path.waypoints[i].p = 0;
        path.waypoints[i].q = 0;
        path.waypoints[i].r = 0;
        path.waypoints[i].flag = 0;
    }

    // Setup Segment #2
    dx = x3.d[0] - x2.d[0];
    dy = x3.d[1] - x2.d[1];
    dz = x3.d[2] - x2.d[2];
    d_len = sqrt(dx*dx + dy*dy + dz*dz);
    quintic_spline_1d_t q_spline_2  = make_1d_quintic_spline(d_len, t3 - t2);

    // Run through Segment #2
    for (int i=0; i < num_pts_per_spline; i++) 
    {
        // 1) Get 1d position
        t_curr = ( ((double) i) / ((double) (num_pts_per_spline-1))) * (t3 - t2);
        s_curr = compute_spline_position(&q_spline_2, t_curr);

        // 2) Convert to 3d position
        

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            x_curr = (s_curr / d_len) * dx  + x2.d[0];
            y_curr = (s_curr / d_len) * dy  + x2.d[1];
            z_curr = (s_curr / d_len) * dz  + x2.d[2];
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            x_curr = x2.d[0];
            y_curr = x2.d[1];
            z_curr = x2.d[2];
        }

        // 3) Write to the path
        int j = i + num_pts_per_spline;
        path.waypoints[j].t = t_curr + t2;
        path.waypoints[j].x = x_curr;
        path.waypoints[j].y = y_curr;
        path.waypoints[j].z = z_curr;
        path.waypoints[j].xd = 0;
        path.waypoints[j].yd = 0;
        path.waypoints[j].zd = 0;
        path.waypoints[j].roll = 0;
        path.waypoints[j].pitch = 0;
        path.waypoints[j].yaw = state_estimate.continuous_yaw;
        path.waypoints[j].p = 0;
        path.waypoints[j].q = 0;
        path.waypoints[j].r = 0;
        path.waypoints[j].flag = 0;
    }

    // for (unsigned int i=0; i < path.len; i++) {
    //     printf("|%+5.2f|%+5.2f|%+5.2f|%+5.2f|\n",
    //         path.waypoints[i].t, path.waypoints[i].x, path.waypoints[i].y, path.waypoints[i].z);
    // }


    path.initialized = 1;
    return 0;
}