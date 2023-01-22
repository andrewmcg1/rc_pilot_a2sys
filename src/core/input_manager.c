/**
 * @file input_manager.c
 */

#include <errno.h>
#include <math.h>  // for fabs
#include <stdio.h>
#include <unistd.h>

#include <rc/dsm.h>
#include <rc/math/other.h>
#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <input_manager.h>
#include <rc_pilot_defs.h>
#include <settings.h>
#include <state_estimator.h>
#include <thread_defs.h>
#include <flight_mode.h>

user_input_t user_input;  // extern variable in input_manager.h

static pthread_t input_manager_thread;
static arm_state_t kill_switch = DISARMED;  // raw kill switch on the radio

static bool enabled_emergency_land = false;


double deadzone(double in, double zone)
{
    if (zone <= 0.0) return in;
    if (fabs(in) <= zone) return 0.0;
    if (in > 0.0)
        return ((in - zone) / (1.0 - zone));
    else
        return ((in + zone) / (1.0 - zone));
}

/**
 * @brief      blocking function that returns after arming sequence is complete
 *
 * @return     0 if successful or already armed, -1 if exiting or problem
 */
static int __wait_for_arming_sequence()
{
    // already armed, just return. Should never do this in normal operation though
    if (user_input.requested_arm_mode == ARMED) return 0;

ARM_SEQUENCE_START:
    // wait for feedback controller to have started
    while (fstate.initialized == 0)
    {
        rc_usleep(100000);
        if (rc_get_state() == EXITING) return 0;
    }
    // wait for level
    while (fabs(state_estimate.roll) > ARM_TIP_THRESHOLD ||
           fabs(state_estimate.pitch) > ARM_TIP_THRESHOLD)
    {
        rc_usleep(100000);
        if (rc_get_state() == EXITING) return 0;
    }
    // wait for kill switch to be switched to ARMED
    while (kill_switch == DISARMED)
    {
        rc_usleep(100000);
        if (rc_get_state() == EXITING) return 0;
    }
    // wait for throttle up
    while (rc_dsm_ch_normalized(settings.dsm_thr_ch) * settings.dsm_thr_pol < 0.9)
    {
        rc_usleep(100000);
        if (rc_get_state() == EXITING) return 0;
        if (kill_switch == DISARMED) goto ARM_SEQUENCE_START;
    }
    // wait for throttle down
    while (rc_dsm_ch_normalized(settings.dsm_thr_ch) * settings.dsm_thr_pol > 0.1)
    {
        rc_usleep(100000);
        if (rc_get_state() == EXITING) return 0;
        if (kill_switch == DISARMED) goto ARM_SEQUENCE_START;
    }

    // final check of kill switch and level before arming
    if (kill_switch == DISARMED) goto ARM_SEQUENCE_START;
    if (fabs(state_estimate.roll) > ARM_TIP_THRESHOLD ||
        fabs(state_estimate.pitch) > ARM_TIP_THRESHOLD)
    {
        goto ARM_SEQUENCE_START;
    }
    return 0;
}

void new_dsm_data_callback()
{
    double new_thr, new_roll, new_pitch, new_yaw, new_mode, new_kill;

    // Read normalized (+-1) inputs from RC radio stick and multiply by
    // polarity setting so positive stick means positive setpoint
    new_thr = rc_dsm_ch_normalized(settings.dsm_thr_ch) * settings.dsm_thr_pol;
    new_roll = rc_dsm_ch_normalized(settings.dsm_roll_ch) * settings.dsm_roll_pol;
    new_pitch = rc_dsm_ch_normalized(settings.dsm_pitch_ch) * settings.dsm_pitch_pol;
    new_yaw =
        deadzone(rc_dsm_ch_normalized(settings.dsm_yaw_ch) * settings.dsm_yaw_pol, YAW_DEADZONE);
    new_mode = rc_dsm_ch_normalized(settings.dsm_mode_ch) * settings.dsm_mode_pol;

    // kill mode behaviors
    switch (settings.dsm_kill_mode)
    {
        case DSM_KILL_DEDICATED_SWITCH:
            new_kill = rc_dsm_ch_normalized(settings.dsm_kill_ch) * settings.dsm_kill_pol;
            // determine the kill state
            if (new_kill <= 0.5)
            {
                kill_switch = DISARMED;
                user_input.kill_switch = 0;
                user_input.requested_arm_mode = DISARMED;
            }
            else
            {
                user_input.kill_switch = 1;
                kill_switch = ARMED;
            }
            break;
        case DSM_KILL_NEGATIVE_THROTTLE:
            if (new_thr <= -1.1)
            {
                kill_switch = DISARMED;
                user_input.requested_arm_mode = DISARMED;
            }
            else
            {
                kill_switch = ARMED;
            }
            break;

        default:
            fprintf(stderr, "ERROR in input manager, unhandled settings.dsm_kill_mode\n");
            return;
    }

    // saturate the sticks to avoid possible erratic behavior
    // throttle can drop below -1 so extend the range for thr
    rc_saturate_double(&new_thr, -1.0, 1.0);
    rc_saturate_double(&new_roll, -1.0, 1.0);
    rc_saturate_double(&new_pitch, -1.0, 1.0);
    rc_saturate_double(&new_yaw, -1.0, 1.0);

    // grab previous flight mode before overwriting
    user_input.prev_flight_mode = user_input.flight_mode;

    // pick flight mode
    switch (settings.num_dsm_modes)
    {
        case 1:
            user_input.flight_mode = settings.flight_mode_1;
            break;
        case 2:
            // switch will either range from -1 to 1 or 0 to 1.
            // in either case it's safe to use +0.5 as the cutoff
            if (new_mode > 0.5)
                user_input.flight_mode = settings.flight_mode_2;
            else
                user_input.flight_mode = settings.flight_mode_1;
            break;
        case 3:
            // 3-position switch will have the positions -1, 0, 1 when
            // calibrated correctly. checking +- 0.5 is a safe cutoff
            if (new_mode > 0.5)
                user_input.flight_mode = settings.flight_mode_3;
            else if (new_mode < -0.5)
                user_input.flight_mode = settings.flight_mode_1;
            else
                user_input.flight_mode = settings.flight_mode_2;
            break;
        default:
            fprintf(stderr, "ERROR, in input_manager, num_dsm_modes must be 1,2 or 3\n");
            fprintf(stderr, "selecting flight mode 1\n");
            user_input.flight_mode = settings.flight_mode_1;
            break;
    }


    //If we need MOCAP and it is no longer available and the user has indicated
    //they want an emergency landing in case of dropouts, 
    //then manage switching into and out of to OPEN_LOOP_DESCENT
	if( mode_needs_mocap(user_input.flight_mode) && 
        settings.enable_mocap_dropout_emergency_land)
	{
        //Compute time since last MOCAP packet was received (in milliseconds)
        // Each value here must be cast to a double before diving. 
        // Although I expect this to implicitly cast, the testing shows that OPEN_LOOP_DESCENT "randomly" triggers at inappropriate times
        double ms_since_mocap = ((double)rc_nanos_since_epoch() - (double)state_estimate.xbee_time_received_ns) / 1e6;

        //If MOCAP has been out for too long, then enable emergency landing
        if( enabled_emergency_land==false && 
            ms_since_mocap >= settings.mocap_dropout_timeout_ms &&
            user_input.requested_arm_mode == ARMED)
        {
            //Enable Emergency landing. 
            //This extra check is done so that we don't exit emergency landing if MOCAP becomes available suddenly
            enabled_emergency_land = true;

            fprintf(stderr, "ENABLE EMERGENCY LANDING MODE\n");
            fprintf(stderr, "\tThrottle %lf \n", settings.dropout_z_throttle);
            fprintf(stderr, "\tMOCAP has been gone for %lfms which exeeds the limit %lfms\n", ms_since_mocap, settings.mocap_dropout_timeout_ms);
        }

        //Force flight mode to be OPEN_LOOP_DESCENT as long as emergency landing is enables
        if(enabled_emergency_land)
        {
            user_input.flight_mode = OPEN_LOOP_DESCENT;
        }
        
	}
    else
    {
        // if(enabled_emergency_land == true)
        // {
        //     fprintf(stderr, "DISABLE EMERGENCY LANDING MODE\n");
        // }

        //Turn off emergency landing if we enter a mode that does NOT need MOCAP
        enabled_emergency_land = false;
    }
    
    // fill in sticks
    if (user_input.requested_arm_mode == ARMED)
    {
        user_input.thr_stick = new_thr;
        user_input.roll_stick = new_roll;
        user_input.pitch_stick = new_pitch;
        user_input.yaw_stick = new_yaw;
        user_input.requested_arm_mode = kill_switch;
    }
    else
    {
        // during arming sequence keep sticks zeroed
        user_input.thr_stick = 0.0;
        user_input.roll_stick = 0.0;
        user_input.pitch_stick = 0.0;
        user_input.yaw_stick = 0.0;
    }

    if (user_input.input_active == 0)
    {
        user_input.input_active = 1;  // flag that connection has come back online
        printf("DSM CONNECTION ESTABLISHED\n");
    }
    return;
}

void dsm_disconnect_actions()
{
    user_input.thr_stick = 0.0;
    user_input.roll_stick = 0.0;
    user_input.pitch_stick = 0.0;
    user_input.yaw_stick = 0.0;
    kill_switch = DISARMED;
    user_input.requested_arm_mode = DISARMED;
    fprintf(stderr, "DSM DISCONNECT ACTIONS\n");
}
void dsm_disconnect_callback(void)
{
    // Preliminary loss of connection actions
    user_input.input_active = 0;
    fprintf(stderr, "LOST DSM CONNECTION\n");
}

typedef struct disconnect_timer_t
{
    uint64_t time_start;
    int active;
} disconnect_timer_t;

void* input_manager(__attribute__((unused)) void* ptr)
{
    disconnect_timer_t disconnect_timer;
    disconnect_timer.active = 0;

    user_input.initialized = 1;
    // wait for first packet
    while (rc_get_state() != EXITING)
    {
        if (user_input.input_active) break;
        rc_usleep(1000000 / INPUT_MANAGER_HZ);
    }

    // not much to do since the DSM callbacks to most of it. Later some
    // logic to handle other inputs such as mavlink/bluetooth/wifi
    while (rc_get_state() != EXITING)
    {
        // if the core got disarmed, wait for arming sequence
        if (user_input.requested_arm_mode == DISARMED)
        {
            __wait_for_arming_sequence();
            // user may have pressed the pause button or shut down while waiting
            // check before continuing
            if (rc_get_state() != RUNNING)
            {
                continue;
            }
            else
            {
                user_input.requested_arm_mode = ARMED;
                // printf("\n\nDSM ARM REQUEST\n\n");
            }
        }

        // check if disconnected
        if (user_input.input_active == 0)
        {
            // check if timer started
            if (disconnect_timer.active == 0)
            {
                // if timer has not started, set start time to
                // current time and set the timer to active
                disconnect_timer.time_start = rc_nanos_since_epoch();
                disconnect_timer.active = 1;
            }
            else
            {
                // if the timer has started, check if more that dsm_timout seconds
                // (dsm_timout_ms * 1e6) have elapsed
                if (rc_nanos_since_epoch() >
                    (disconnect_timer.time_start + settings.dsm_timeout_ms * 1e6))
                {
                    // if too much time has elapsed, perform disconnect actions
                    dsm_disconnect_actions();
                    disconnect_timer.active = 0;  // deactivate timer for next disconnect
                }
            }
        }
        else
        {
            // if user input is reestablished, turn the timer off
            disconnect_timer.active = 0;
        }
        // wait
        rc_usleep(1000000 / INPUT_MANAGER_HZ);
    }
    return NULL;
}

int input_manager_init()
{
    enabled_emergency_land = false;
    user_input.initialized = 0;
    int i;
    // start dsm hardware
    if (rc_dsm_init() == -1)
    {
        fprintf(stderr, "ERROR in input_manager_init, failed to initialize dsm\n");
        return -1;
    }
    rc_dsm_set_disconnect_callback(dsm_disconnect_callback);
    rc_dsm_set_callback(new_dsm_data_callback);
    // start thread
    if (rc_pthread_create(
            &input_manager_thread, &input_manager, NULL, SCHED_FIFO, INPUT_MANAGER_PRI) == -1)
    {
        fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
        return -1;
    }
    // wait for thread to start
    for (i = 0; i < 50; i++)
    {
        if (user_input.initialized) return 0;
        rc_usleep(50000);
    }
    fprintf(stderr, "ERROR in input_manager_init, timeout waiting for thread to start\n");
    return -1;
}

int input_manager_cleanup()
{
    if (user_input.initialized == 0)
    {
        fprintf(stderr, "WARNING in input_manager_cleanup, was never initialized\n");
        return -1;
    }
    // wait for the thread to exit
    if (rc_pthread_timed_join(input_manager_thread, NULL, INPUT_MANAGER_TOUT) == 1)
    {
        fprintf(stderr, "WARNING: in input_manager_cleanup, thread join timeout\n");
        return -1;
    }
    // stop dsm
    rc_dsm_cleanup();
    return 0;
}
