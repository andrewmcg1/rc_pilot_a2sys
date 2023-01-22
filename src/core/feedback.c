/**
 * @file feedback.c
 *
 */

#include <math.h>
#include <rc/led.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/other.h>
#include <rc/math/quaternion.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <stdio.h>

#include <controller.h>
#include <feedback.h>
#include <log_manager.h>
#include <mix.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>
#include <thrust_map.h>

#define TWO_PI (M_PI * 2.0)

feedback_state_t fstate;  // extern variable in feedback.h

static int __send_motor_stop_pulse(void)
{
    int i;
    if (settings.num_rotors > 8)
    {
        printf("ERROR: set_motors_to_idle: too many rotors\n");
        return -1;
    }
    for (i = 0; i < settings.num_rotors; i++)
    {
        fstate.m[i] = -0.1;
        rc_servo_send_esc_pulse_normalized(i + 1, -0.1);
    }
    return 0;
}

int feedback_disarm(void)
{
    fstate.arm_state = DISARMED;

#ifndef OFFBOARD_TEST
    // set LEDs
    rc_led_set(RC_LED_RED, 1);
    rc_led_set(RC_LED_GREEN, 0);
#endif

    return 0;
}

int feedback_arm(void)
{
    if (fstate.arm_state == ARMED)
    {
        printf("WARNING: trying to arm when controller is already armed\n");
        return -1;
    }

    if (fstate.fail_state == FAILED)
    {
        printf("WARNING: Trying to arm the controller in a fail state\n");
        return -1;
    }

    // start a new log file every time controller is armed, this may take some
    // time so do it before touching anything else
    if (settings.enable_logging) log_manager_init();

    // get the current time
    fstate.arm_time_ns = rc_nanos_since_epoch();

    // reset the index
    fstate.loop_index = 0;

    // Reset controllers, clears integrators and prefills outputs as necessary
    controller_reset();

    // set LEDs
    rc_led_set(RC_LED_RED, 0);
    rc_led_set(RC_LED_GREEN, 1);
    // last thing is to flag as armed
    fstate.arm_state = ARMED;
    return 0;
}

int feedback_init(void)
{
    controller_init();

    // make sure everything is disarmed them start the ISR
    feedback_disarm();
    fstate.fail_state = SAFE;  // Assume quad is starting out safe
    fstate.initialized = 1;

    return 0;
}

int feedback_march(void)
{
    int i;
    double u[6], mot[8];

    // Disarm if rc_state is somehow paused without disarming the controller.
    // This shouldn't happen if other threads are working properly.
    if (rc_get_state() != RUNNING && fstate.arm_state == ARMED)
    {
        feedback_disarm();
    }

    // check for a tipover
    if (fabs(state_estimate.roll) > TIP_ANGLE || fabs(state_estimate.pitch) > TIP_ANGLE)
    {
        feedback_disarm();
        if(fstate.fail_state != FAILED)
            printf("\n TIPOVER DETECTED \n");
        fstate.fail_state = FAILED;
    }
    else
    {
        // If the tipover is corrected and the controller is set to disarmed, fail state is safe
        if (fstate.fail_state == FAILED && user_input.requested_arm_mode == DISARMED)
        {
            fstate.fail_state = SAFE;
        }
    }

    // if not running or not armed, keep the motors in an idle state
    if (rc_get_state() != RUNNING || fstate.arm_state == DISARMED)
    {
        __send_motor_stop_pulse();
        if (settings.log_only_while_armed) {
            log_manager_cleanup();
        }
        return 0;
    }

    // We are about to start marching the individual SISO controllers forward.
    // Start by zeroing out the motors signals then add from there.
    for (i = 0; i < 8; i++) mot[i] = 0.0;
    for (i = 0; i < 6; i++) u[i] = 0.0;

    feedback_controller(u, mot);

    /***************************************************************************
     * Send ESC motor signals immediately at the end of the control loop
     ***************************************************************************/
    for (i = 0; i < settings.num_rotors; i++)
    {
        rc_saturate_double(&mot[i], 0.0, 1.0);
        fstate.m[i] = map_motor_signal(mot[i]);

        // final saturation just to take care of possible rounding errors
        // this should not change the values and is probably excessive
        rc_saturate_double(&fstate.m[i], 0.0, 1.0);

        // finally send pulses!
        rc_servo_send_esc_pulse_normalized(i + 1, fstate.m[i]);
    }

    /***************************************************************************
     * Final cleanup, timing, and indexing
     ***************************************************************************/
    // Load control inputs into cstate for viewing by outside threads
    for (i = 0; i < 6; i++) fstate.u[i] = u[i];
    // keep track of loops since arming
    fstate.loop_index++;
    // log us since arming, mostly for the log
    fstate.last_step_ns = rc_nanos_since_epoch();

    return 0;
}

int feedback_cleanup(void)
{
#ifndef OFFBOARD_TEST
    __send_motor_stop_pulse();
#endif
    return 0;
}
