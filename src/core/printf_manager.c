/**
 * @file printf_manager.c
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <feedback.h>
#include <input_manager.h>
#include <naza_gps.h>
#include <printf_manager.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>
#include <thread_defs.h>
#include <xbee_receive.h>
#include <pni_rm3100.h>
#include <vl53l1x.h>

static pthread_t printf_manager_thread;
static int initialized = 0;

const char* const colours[] = {KYEL, KCYN, KGRN, KMAG};
const int num_colours = 4;  // length of above array
int current_colour = 0;

/**
 * @brief      { function_description }
 *
 * @return     string with ascii colour code
 */
static const char* __next_colour()
{
    // if reached the end of the colour list, loop around
    if (current_colour >= (num_colours - 1))
    {
        current_colour = 0;
        return colours[num_colours - 1];
    }
    // else increment counter and return
    current_colour++;
    return colours[current_colour - 1];
}

static void __reset_colour()
{
    current_colour = 0;
}

static int __print_header()
{
    int i;

    printf("\n");
    __reset_colour();
    if (settings.printf_arm)
    {
        printf("  arm   |");
    }
    if (settings.printf_tracking)
    {
        printf("%s gps|moc|", __next_colour());
    }
    if (settings.printf_altitude)
    {
        printf("%s alt(m) |altdot|", __next_colour());
    }
    if (settings.printf_battery)
    {
        printf("%s batt (V)|", __next_colour());
    }
    if (settings.printf_rpy)
    {
        printf("%s roll|pitch| yaw |", __next_colour());
    }
    if (settings.printf_sticks)
    {
        printf("%s  kill  | thr |roll |pitch| yaw |", __next_colour());
    }
    if (settings.printf_setpoint)
    {
        printf("%s sp_x| sp_y| sp_z| sp_r| sp_p| sp_y|", __next_colour());
    }
    if (settings.printf_u)
    {
        printf("%s U0X | U1Y | U2Z | U3r | U4p | U5y |", __next_colour());
    }
    if (settings.printf_xbee && settings.printf_xbee_velocities)
    {
        printf("%s x_xb | y_xb | z_xb | xdot_xb | ydot_xb | zdot_xb | qx_xb | qy_xb | qz_xb | qw_xb | sm_xb |", __next_colour());
    }
    else if (settings.printf_xbee && !settings.printf_xbee_velocities)
    {
        printf("%s x_xb | y_xb | z_xb | qx_xb | qy_xb | qz_xb | qw_xb | sm_xb |", __next_colour());
    }
    if (settings.printf_gps)
    {
        printf("%sgps_lat|gps_lon|gps_ele|gps_fix|gps_valid|", __next_colour());
    }
    if (settings.printf_rm3100)
    {
        printf("%srm3100_x|rm3100_y|rm3100_z|vec_norm|", __next_colour());
    }
    if (settings.printf_vl53l1x)
    {
        printf("%svl53l1x|", __next_colour());
    }
    if (settings.printf_motors)
    {
        printf("%s", __next_colour());
        for (i = 0; i < settings.num_rotors; i++)
        {
            printf("  M%d |", i + 1);
        }
    }
    if (settings.printf_magnetom)
    {
        printf("%s mag_x | mag_y | mag_z |mag_nrm|", __next_colour());
    }
    printf(KNRM);
    if (settings.printf_mode)
    {
        printf("   MODE ");
    }

    printf("\n");
    fflush(stdout);
    return 0;
}

static void* __printf_manager_func(__attribute__((unused)) void* ptr)
{
    int i;
    initialized = 1;
    printf("\nTurn your transmitter kill switch to arm.\n");
    printf("Then move throttle UP then DOWN to arm controller\n\n");

    // turn off linewrap to avoid runaway prints
    printf(WRAP_DISABLE);

    // print the header
    __print_header();

    // sleep so state_estimator can run first
    rc_usleep(100000);

    while (rc_get_state() != EXITING)
    {
        __reset_colour();

        printf("\r");
        
        if (settings.printf_arm)
        {
            if (fstate.arm_state == ARMED)
            {
                printf("%s ARMED %s |", KRED, KNRM);
            }
            else
            {
                printf("%sDISARMED%s|", KGRN, KNRM);
            }
        }
        if (settings.printf_tracking)
        {
            printf("%s %.2i | %i |", __next_colour(), gps_data.sat, xbeeMsg.trackingValid);
        }
        if (settings.printf_altitude)
        {
            printf("%s%+5.2f |%+5.2f |", __next_colour(), state_estimate.alt_bmp,
                state_estimate.alt_bmp_vel);
        }
        if (settings.printf_battery)
        {
            printf("%s%9.2f|", __next_colour(), state_estimate.v_batt_lp);
        }
        if (settings.printf_rpy)
        {
            printf(KCYN);
            printf("%s%+5.2f|%+5.2f|%+5.2f|", __next_colour(), state_estimate.roll,
                state_estimate.pitch, state_estimate.continuous_yaw);
        }
        if (settings.printf_sticks)
        {
            if (user_input.kill_switch == 1)
            {
                printf("%s ARMED  ", KRED);
            }
            else
            {
                printf("%sDISARMED", KGRN);
            }
            printf(KGRN);
            printf("%s|%+5.2f|%+5.2f|%+5.2f|%+5.2f|", __next_colour(), user_input.thr_stick,
                user_input.roll_stick, user_input.pitch_stick, user_input.yaw_stick);
        }
        if (settings.printf_setpoint)
        {
            printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|", __next_colour(), 
                setpoint.X, setpoint.Y, setpoint.Z, 
                setpoint.roll, setpoint.pitch, setpoint.yaw);
        }
        if (settings.printf_u)
        {
            printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|", __next_colour(), fstate.u[0],
                fstate.u[1], fstate.u[2], fstate.u[3], fstate.u[4], fstate.u[5]);
        }
        if (settings.printf_xbee && settings.printf_xbee_velocities)
        {
            printf("%s%+6.2f|%+6.2f|%+6.2f|%+9.2f|%+9.2f|%+9.2f|%+7.2f|%+7.2f|%+7.2f|%+7.2f|  %2X   |", __next_colour(),
                xbeeMsg.x, xbeeMsg.y, xbeeMsg.z, state_estimate.X_dot, state_estimate.Y_dot,
                state_estimate.Z_dot, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz, xbeeMsg.qw,
                xbeeMsg.sm_event);
        }
        else if (settings.printf_xbee && !settings.printf_xbee_velocities)
        {
            printf("%s%+6.2f|%+6.2f|%+6.2f|%+7.2f|%+7.2f|%+7.2f|%+7.2f|  %2X   |", __next_colour(),
                xbeeMsg.x, xbeeMsg.y, xbeeMsg.z, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz, xbeeMsg.qw,
                xbeeMsg.sm_event);
        }
        if (settings.printf_gps)
        {
            printf("%s%+7.2f|%+7.2f|%+7.2f|%7.2d|  %7.1d|", __next_colour(), 
                gps_data.lla.lat, gps_data.lla.lon, gps_data.lla.alt, gps_data.fix, gps_data.gps_valid);
        }
        if (settings.printf_rm3100)
        {
            //If applying calibration, the print the calibrated values to terminal
            //Otherwise, print raw magnetometer values
            if(settings.apply_rm3100_calibration)
            {
                printf("%s%+8.3lf|%+8.3lf|%+8.3lf|%+8.3lf|", __next_colour(),
                    rm3100_data_extern.cal_mag[0], rm3100_data_extern.cal_mag[1], rm3100_data_extern.cal_mag[2],
                    sqrt(pow(rm3100_data_extern.cal_mag[0], 2) + pow(rm3100_data_extern.cal_mag[1], 2) + pow(rm3100_data_extern.cal_mag[2], 2) )
                  );
            }
            else{
                printf("%s%+8.3lf|%+8.3lf|%+8.3lf|%+8.3lf|", __next_colour(),
                    rm3100_data_extern.mag[0], rm3100_data_extern.mag[1], rm3100_data_extern.mag[2],
                    sqrt(pow(rm3100_data_extern.mag[0], 2) + pow(rm3100_data_extern.mag[1], 2) + pow(rm3100_data_extern.mag[2], 2) )
                  );
            }
        }
        if (settings.printf_vl53l1x)
        {
            printf("%s%+7.3lf|", __next_colour(), vl53l1_data_extern.distance_m);
        }
        if (settings.printf_motors)
        {
            printf("%s", __next_colour());
            for (i = 0; i < settings.num_rotors; i++)
            {
                printf("%+5.2f|", fstate.m[i]);
            }
        }
        if (settings.printf_magnetom)
        {
            printf("%s%+7.2f|%+7.2f|%+7.2f|%+7.2f|", __next_colour(), state_estimate.mag[0],
                state_estimate.mag[1], state_estimate.mag[2],
                sqrt(pow(state_estimate.mag[0], 2) + pow(state_estimate.mag[1], 2) +
                     pow(state_estimate.mag[2], 2)));
        }
        printf(KNRM);
        if (settings.printf_mode)
        {
            print_flight_mode(user_input.flight_mode);
        }

        fflush(stdout);
        rc_usleep(1000000 / PRINTF_MANAGER_HZ);
    }

    // put linewrap back on
    printf(WRAP_ENABLE);

    return NULL;
}

int printf_init()
{
    if (rc_pthread_create(&printf_manager_thread, __printf_manager_func, NULL, SCHED_FIFO,
            PRINTF_MANAGER_PRI) == -1)
    {
        fprintf(stderr, "ERROR in start_printf_manager, failed to start thread\n");
        return -1;
    }
    rc_usleep(50000);
    return 0;
}

int printf_cleanup()
{
    int ret = 0;
    if (initialized)
    {
        // wait for the thread to exit
        ret = rc_pthread_timed_join(printf_manager_thread, NULL, PRINTF_MANAGER_TOUT);
        if (ret == 1)
            fprintf(stderr, "WARNING: printf_manager_thread exit timeout\n");
        else if (ret == -1)
            fprintf(stderr, "ERROR: failed to join printf_manager thread\n");
    }
    initialized = 0;
    return ret;
}

int print_flight_mode(flight_mode_t mode)
{
    switch (mode)
    {
        case TEST_BENCH_4DOF:
            printf("%sTEST_BENCH_4DOF%s", KYEL, KNRM);
            return 0;
        case TEST_BENCH_6DOF:
            printf("%sTEST_BENCH_6DOF%s", KYEL, KNRM);
            return 0;
        case RP_TEST_DIRECT_THROTTLE:
            printf("%sRP_TEST_DIR_THR%s", KYEL, KNRM);
            return 0;
        case MANUAL:
            printf("%sMANUAL         %s", KCYN, KNRM);
            return 0;
        case ALT_HOLD:
            printf("%sALT_HOLD       %s", KBLU, KNRM);
            return 0;
        case AUTONOMOUS:
            printf("%sAUTONOMOUS     %s", KBLU, KNRM);
            return 0;
        case OPEN_LOOP_DESCENT:
            printf("%sOPN_LOOP_DESCNT%s", KBLU, KNRM);
            return 0;
        case LOITER:
            printf("%sLOITER         %s", KBLU, KNRM);
            return 0;
        case LOITER_RSP:
            printf("%sLOITER_RSP     %s", KBLU, KNRM);
            return 0;
        case ACRO:
            printf("%sACRO           %s", KBLU, KNRM);
            return 0;
        case TEST_BENCH_DIRECT_Z_ACC:
            printf("%sTB_DIRECT_Z_ACC%s", KBLU, KNRM);
            return 0;
        case TEST_BENCH_DIRECT_Z_VEL:
            printf("%sTB_DIRECT_Z_VEL%s", KBLU, KNRM);
            return 0;
        default:
            fprintf(stderr, "ERROR in print_flight_mode, unknown flight mode\n");
            return -1;
    }
}