#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <controller.h>
#include <rc/math.h>
#include <rc/time.h>
#include <settings.h>
#include <state_machine.h>
#include <flight_mode.h>

// Flight Mode Startup Delay variables
static uint64_t time_fm_started_ns;
static uint64_t time_fm_needs_to_startup_ns = 1e9;
bool fm_starting_up = false;

// filters
static rc_filter_t D_roll_rate_pd =  RC_FILTER_INITIALIZER;
static rc_filter_t D_roll_rate_i =   RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch_rate_pd = RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch_rate_i =  RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw_rate_pd =   RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw_rate_i =    RC_FILTER_INITIALIZER;
static rc_filter_t D_roll_pd =       RC_FILTER_INITIALIZER;
static rc_filter_t D_roll_i =        RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch =         RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw =           RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_pd =       RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_i =        RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_pd =       RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_i =        RC_FILTER_INITIALIZER;
static rc_filter_t D_Zdot_pd =       RC_FILTER_INITIALIZER;
static rc_filter_t D_Zdot_i =        RC_FILTER_INITIALIZER;
static rc_filter_t D_X_pd =          RC_FILTER_INITIALIZER;
static rc_filter_t D_X_i =           RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_pd =          RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_i =           RC_FILTER_INITIALIZER;
static rc_filter_t D_Z_pd =          RC_FILTER_INITIALIZER;
static rc_filter_t D_Z_i =           RC_FILTER_INITIALIZER;

static void __rpy_init(void)
{
    // get controllers from settings
    rc_filter_duplicate(&D_roll_rate_pd, settings.roll_rate_controller_pd);
    rc_filter_duplicate(&D_roll_rate_i,  settings.roll_rate_controller_i);

    rc_filter_duplicate(&D_pitch_rate_pd, settings.pitch_rate_controller_pd);
    rc_filter_duplicate(&D_pitch_rate_i,  settings.pitch_rate_controller_i);

    rc_filter_duplicate(&D_yaw_rate_pd, settings.yaw_rate_controller_pd);
    rc_filter_duplicate(&D_yaw_rate_i,  settings.yaw_rate_controller_i);

    rc_filter_duplicate(&D_roll_pd,  settings.roll_controller_pd);
    rc_filter_duplicate(&D_roll_i,  settings.roll_controller_i);
    rc_filter_duplicate(&D_pitch, settings.pitch_controller);
    rc_filter_duplicate(&D_yaw,   settings.yaw_controller);

#ifdef DEBUG
    printf("ROLL CONTROLLER:\n");
    rc_filter_print(D_roll_pd);
    printf("PITCH CONTROLLER:\n");
    rc_filter_print(D_pitch);
    printf("YAW CONTROLLER:\n");
    rc_filter_print(D_yaw);
#endif
}

static void __xyz_init(void)
{
    rc_filter_duplicate(&D_Xdot_pd, settings.horiz_vel_ctrl_pd);
    rc_filter_duplicate(&D_Xdot_i,  settings.horiz_vel_ctrl_i);

    rc_filter_duplicate(&D_Ydot_pd, settings.horiz_vel_ctrl_pd);
    rc_filter_duplicate(&D_Ydot_i,  settings.horiz_vel_ctrl_i);

    rc_filter_duplicate(&D_Zdot_pd, settings.altitude_rate_controller_pd);
    rc_filter_duplicate(&D_Zdot_i,  settings.altitude_rate_controller_i);
   
    rc_filter_duplicate(&D_X_pd, settings.horiz_pos_ctrl_pd);
    rc_filter_duplicate(&D_X_i, settings.horiz_pos_ctrl_i);

    rc_filter_duplicate(&D_Y_pd, settings.horiz_pos_ctrl_pd);
    rc_filter_duplicate(&D_Y_i, settings.horiz_pos_ctrl_i);

    rc_filter_duplicate(&D_Z_pd, settings.altitude_controller_pd);
    rc_filter_duplicate(&D_Z_i,  settings.altitude_controller_i);
}

static void __zero_out_feedforward_terms()
{
    setpoint.roll_dot_ff = 0;
    setpoint.pitch_dot_ff = 0;
    setpoint.yaw_dot_ff = 0;
    setpoint.roll_ff = 0;
    setpoint.pitch_ff = 0;
    setpoint.Z_dot_ff = 0;
    setpoint.X_dot_ff = 0; 
    setpoint.Y_dot_ff = 0; 
}

static void __assign_setpoints_and_enable_loops()
{
    // Zero out feedforward terms so unexpected things don't happen
    __zero_out_feedforward_terms();

    switch (user_input.flight_mode)
    {
        case TEST_BENCH_4DOF:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0;
            setpoint.en_rpy_rate_ctrl = 0;
            setpoint.en_rpy_ctrl = 0;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints
            setpoint.roll_throttle = user_input.roll_stick;
            setpoint.pitch_throttle = user_input.pitch_stick;
            setpoint.yaw_throttle = user_input.yaw_stick;
            setpoint.X_throttle = 0.0;
            setpoint.Y_throttle = 0.0;
            setpoint.Z_throttle = -user_input.thr_stick;
            break;

        case TEST_BENCH_6DOF:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 1;
            setpoint.en_rpy_rate_ctrl = 0;
            setpoint.en_rpy_ctrl = 0;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints
            setpoint.roll_throttle = 0.0;
            setpoint.pitch_throttle = 0.0;
            setpoint.yaw_throttle = user_input.yaw_stick;
            setpoint.X_throttle = -user_input.pitch_stick;
            setpoint.Y_throttle = user_input.roll_stick;
            setpoint.Z_throttle = -user_input.thr_stick;
            break;

        case RP_TEST_DIRECT_THROTTLE:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0;
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints
            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            setpoint.yaw_throttle = user_input.yaw_stick;
            setpoint.X_throttle = 0.0;
            setpoint.Y_throttle = 0.0;
            setpoint.Z_throttle = -user_input.thr_stick;
            break;

        case MANUAL:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0; //(settings.dof == 6);
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints

            // Handle Yaw Transition from Mocap Modes to DIRECT_THROTTLE_4DOF
            if (mode_needs_mocap(user_input.prev_flight_mode))
            {
                setpoint.yaw = state_estimate.mag_heading_continuous;
            }

            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            
            setpoint.Z_throttle = -user_input.thr_stick / (cos(state_estimate.roll) * cos(state_estimate.pitch));
            setpoint_update_yaw();
            break;

        case ALT_HOLD:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0; //(settings.dof == 6);
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints

            // Trigger "startup" delay if we just transitioned
            if (just_transitioned_flight_mode())
            {
                fm_starting_up = true;
                time_fm_started_ns = rc_nanos_since_epoch();
                setpoint_update_XYZ_bumpless();
            }

            // If we've been in fm for long enough we don't need to "startup" anymore
            if (rc_nanos_since_epoch() - time_fm_started_ns >= time_fm_needs_to_startup_ns)
            {
                fm_starting_up = false;
            }
            
            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            if (!fm_starting_up) setpoint_update_Z();
            setpoint_update_yaw();
            break;

        case AUTONOMOUS:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0; //(settings.dof == 6);
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_ctrl = 1;

            // 2) Assign Setpoints
            setpoint_update_setpoint_from_waypoint();

            if (waypoint_state_machine.current_state == STANDBY)
            {
                setpoint.en_rpy_rate_ctrl = 0;
                setpoint.en_rpy_ctrl = 0;
                setpoint.en_Z_ctrl = 0;
                setpoint.en_XY_ctrl = 0;
                setpoint.Z_throttle = 0;
                setpoint.roll_throttle = 0;
                setpoint.pitch_throttle = 0;
            }
            break;

        case OPEN_LOOP_DESCENT:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0;
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints
            setpoint.roll = 0;
            setpoint.pitch = 0;

            //-0.6 is close to hovering. Make the value closer to zero (less negative) if you want to descend faster. 
            //This parameter will also depend on the weight of the vehicle.
            setpoint.Z_throttle = settings.dropout_z_throttle;
            
            setpoint_update_yaw();
            break;

        case LOITER:
        case LOITER_RSP:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0; //(settings.dof == 6);
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_ctrl = 1;

            // 2) Assign Setpoints

            // Trigger "startup" delay if we just transitioned
            if (just_transitioned_flight_mode())
            {
                fm_starting_up = true;
                time_fm_started_ns = rc_nanos_since_epoch();
                setpoint_update_XYZ_bumpless();
            }

            // If we've been in LOITER for long enough we don't need to "startup" anymore
            if (rc_nanos_since_epoch() - time_fm_started_ns >= time_fm_needs_to_startup_ns)
            {
                fm_starting_up = false;
            }

            // Bumpless transfer if starting up, standard loiter control if not
            if (!fm_starting_up)
            {
                if ((user_input.flight_mode == LOITER_RSP) && (setpoint.following_rsp_cmd))
                {
                    // Follow the landing cmd if it's been received
                    setpoint_update_setpoint_from_waypoint();
                }
                else 
                {
                    // Otherwise, follow transmitter sticks for setpoints
                    setpoint_update_loiter();
                }
            } 
            break;

        case ACRO:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0; //(settings.dof == 6);
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 0;
            setpoint.en_Z_ctrl = 0;
            setpoint.en_XY_ctrl = 0;

            // 2) Assign Setpoints
            setpoint.roll_dot =  user_input.roll_stick  * MAX_ROLL_RATE;
            setpoint.pitch_dot = user_input.pitch_stick * MAX_PITCH_RATE;
            setpoint.yaw_dot =   user_input.yaw_stick   * MAX_YAW_RATE;
            setpoint.Z_throttle = -user_input.thr_stick / (cos(state_estimate.roll) * cos(state_estimate.pitch));
            break;

        case TEST_BENCH_DIRECT_Z_ACC:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0;
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_ctrl = 0;

            // 1.5) "Disable" the two other z PID loops besides throttle
            rc_filter_enable_saturation(&D_Zdot_pd, 0, 0);
            rc_filter_enable_saturation(&D_Zdot_i, 0, 0);
            rc_filter_enable_saturation(&D_Z_pd, 0, 0);
            rc_filter_enable_saturation(&D_Z_i, 0, 0);

            // 2) Assign Setpoints
            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            setpoint_update_yaw();
            setpoint.Z_ddot = -1 * (2*user_input.thr_stick - 1) * MAX_Z_ACCELERATION;
            break;
        
        case TEST_BENCH_DIRECT_Z_VEL:
            // 1) Enable PID Loops based on flight mode
            setpoint.en_6dof = 0;
            setpoint.en_rpy_rate_ctrl = 1;
            setpoint.en_rpy_ctrl = 1;
            setpoint.en_Z_ctrl = 1;
            setpoint.en_XY_ctrl = 0;

            // 1.5) "Disable" the position controller
            rc_filter_enable_saturation(&D_Z_pd, 0, 0);
            rc_filter_enable_saturation(&D_Z_i, 0, 0);

            // 2) Assign Setpoints
            setpoint.roll = user_input.roll_stick;
            setpoint.pitch = user_input.pitch_stick;
            setpoint_update_yaw();
            setpoint.Z_dot_ff = -1 * (2*user_input.thr_stick - 1) * MAX_Z_VELOCITY;
            break;

        default:  // should never get here
            fprintf(stderr, "ERROR in setpoint_manager thread, unknown flight mode\n");
            break;

    }  // end switch(user_input.flight_mode)
}

#ifndef OFFBOARD_TEST
static void __run_Z_controller()
{
    // 1) Position -> Velocity
    setpoint.Z_dot = rc_filter_march(&D_Z_pd, setpoint.Z - state_estimate.Z)
                   + rc_filter_march(&D_Z_i,  setpoint.Z - state_estimate.Z)
                   + setpoint.Z_dot_ff;
    rc_saturate_double(&setpoint.Z_dot, -MAX_Z_VELOCITY, MAX_Z_VELOCITY);

    
    if (user_input.flight_mode != TEST_BENCH_DIRECT_Z_ACC)
    {
        // 2) Velocity -> Acceleration
        setpoint.Z_ddot = rc_filter_march(&D_Zdot_pd, setpoint.Z_dot - state_estimate.Z_dot)
                        + rc_filter_march(&D_Zdot_i,  setpoint.Z_dot - state_estimate.Z_dot);
        rc_saturate_double(&setpoint.Z_ddot, -MAX_Z_ACCELERATION, MAX_Z_ACCELERATION);
    }

    // 3) Acceleration -> Throttle
    setpoint.Z_throttle = settings.hover_throttle + setpoint.Z_ddot;
    setpoint.Z_throttle = setpoint.Z_throttle / (cos(state_estimate.roll) * cos(state_estimate.pitch));
}

static void __run_XY_controller()
{
    // 1) Position -> Velocity
    setpoint.X_dot = rc_filter_march(&D_X_pd, setpoint.X - state_estimate.X)
                   + rc_filter_march(&D_X_i, setpoint.X - state_estimate.X)
                   + setpoint.X_dot_ff;
    setpoint.Y_dot = rc_filter_march(&D_Y_pd, setpoint.Y - state_estimate.Y)
                   + rc_filter_march(&D_Y_i, setpoint.Y - state_estimate.Y)
                   + setpoint.Y_dot_ff;
    rc_saturate_double(&setpoint.X_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
    rc_saturate_double(&setpoint.Y_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);

    // 2) Velocity -> Acceleration
    setpoint.X_ddot = rc_filter_march(&D_Xdot_pd, setpoint.X_dot - state_estimate.X_dot)
                    + rc_filter_march(&D_Xdot_i,  setpoint.X_dot - state_estimate.X_dot);
    setpoint.Y_ddot = rc_filter_march(&D_Ydot_pd, setpoint.Y_dot - state_estimate.Y_dot)
                    + rc_filter_march(&D_Ydot_i,  setpoint.Y_dot - state_estimate.Y_dot);
    rc_saturate_double(&setpoint.X_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
    rc_saturate_double(&setpoint.Y_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
    
    // 3) Acceleration -> Lean Angles
    setpoint.roll = ((-sin(state_estimate.continuous_yaw) * setpoint.X_ddot 
                      +cos(state_estimate.continuous_yaw) * setpoint.Y_ddot)
                      / GRAVITY)
                    + setpoint.roll_ff;              
    setpoint.pitch = ((-cos(state_estimate.continuous_yaw) * setpoint.X_ddot 
                       -sin(state_estimate.continuous_yaw) * setpoint.Y_ddot)
                      / GRAVITY)
                    + setpoint.pitch_ff;
    rc_saturate_double(&setpoint.roll, -MAX_ROLL_SETPOINT, MAX_ROLL_SETPOINT);
    rc_saturate_double(&setpoint.pitch, -MAX_PITCH_SETPOINT, MAX_PITCH_SETPOINT);
}


static void __run_attitude_controller()
{
    // 1) Attitude -> Attitude Rate
    setpoint.roll_dot  = rc_filter_march(&D_roll_pd,  setpoint.roll  - state_estimate.roll)
                       + rc_filter_march(&D_roll_i,   setpoint.roll  - state_estimate.roll)
                       + setpoint.roll_dot_ff;
    setpoint.pitch_dot = rc_filter_march(&D_pitch, setpoint.pitch - state_estimate.pitch)
                       + setpoint.pitch_dot_ff;
    setpoint.yaw_dot   = rc_filter_march(&D_yaw,   setpoint.yaw   - state_estimate.continuous_yaw)
                       + setpoint.yaw_dot_ff;
    rc_saturate_double(&setpoint.roll_dot, -MAX_ROLL_RATE, MAX_ROLL_RATE);
    rc_saturate_double(&setpoint.pitch_dot, -MAX_PITCH_RATE, MAX_PITCH_RATE);
    rc_saturate_double(&setpoint.yaw_dot, -MAX_YAW_RATE, MAX_YAW_RATE);
}

static void __run_attitude_rate_controller()
{
    // 1) Attitude Rate -> Torques
    setpoint.roll_throttle  = rc_filter_march(&D_roll_rate_pd,  setpoint.roll_dot  - state_estimate.roll_dot)
                            + rc_filter_march(&D_roll_rate_i,   setpoint.roll_dot  - state_estimate.roll_dot);
    setpoint.pitch_throttle = rc_filter_march(&D_pitch_rate_pd, setpoint.pitch_dot - state_estimate.pitch_dot)
                            + rc_filter_march(&D_pitch_rate_i,  setpoint.pitch_dot - state_estimate.pitch_dot);
    setpoint.yaw_throttle   = rc_filter_march(&D_yaw_rate_pd,   setpoint.yaw_dot   - state_estimate.yaw_dot)
                            + rc_filter_march(&D_yaw_rate_i,    setpoint.yaw_dot   - state_estimate.yaw_dot);    
}

static void __add_throttles_to_mixing_matrix(double* u, double* mot)
{
    double min, max;
    double baseline_throttle = -0.5;
    double extra_throttle = 0.0;

    rc_saturate_double(&setpoint.Z_throttle, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
    if (setpoint.Z_throttle > baseline_throttle) {
        // "low" throttle
        baseline_throttle = setpoint.Z_throttle;
    } else {
        // "high" throttle
        extra_throttle = setpoint.Z_throttle - baseline_throttle;
    }

    // 1) Z (1st pass)
    u[VEC_Z] = baseline_throttle;
    mix_add_input(u[VEC_Z], VEC_Z, mot);

    // 2) Roll (X)
    mix_check_saturation(VEC_ROLL, mot, &min, &max);
    if (max > MAX_ROLL_COMPONENT) max = MAX_ROLL_COMPONENT;
    if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
    u[VEC_ROLL] = setpoint.roll_throttle;
    rc_saturate_double(&u[VEC_ROLL], min, max);
    mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

    // 3) Pitch (Y)
    mix_check_saturation(VEC_PITCH, mot, &min, &max);
    if (max > MAX_PITCH_COMPONENT) max = MAX_PITCH_COMPONENT;
    if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
    u[VEC_PITCH] = setpoint.pitch_throttle;
    rc_saturate_double(&u[VEC_PITCH], min, max);
    mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

    // Z (2nd pass)
    mix_check_saturation(VEC_Z, mot, &min, &max);
    if (max > 0) max = 0;
    if (min < MIN_THRUST_COMPONENT) min = MIN_THRUST_COMPONENT;
    rc_saturate_double(&extra_throttle, min, max);
    mix_add_input(extra_throttle, VEC_Z, mot);
    u[VEC_Z] = baseline_throttle + extra_throttle;

    // 4) Yaw (Z)
    mix_check_saturation(VEC_YAW, mot, &min, &max);
    if (max > MAX_YAW_COMPONENT) max = MAX_YAW_COMPONENT;
    if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
    u[VEC_YAW] = setpoint.yaw_throttle;
    rc_saturate_double(&u[VEC_YAW], min, max);
    mix_add_input(u[VEC_YAW], VEC_YAW, mot);

    // X & Y if 6DOF
    if (setpoint.en_6dof) 
    {
        // 5) X
        mix_check_saturation(VEC_X, mot, &min, &max);
        if (max > MAX_X_COMPONENT) max = MAX_X_COMPONENT;
        if (min < -MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
        u[VEC_X] = setpoint.X_throttle;
        rc_saturate_double(&u[VEC_X], min, max);
        mix_add_input(u[VEC_X], VEC_X, mot);

        // 6) Y
        mix_check_saturation(VEC_Y, mot, &min, &max);
        if (max > MAX_Y_COMPONENT) max = MAX_Y_COMPONENT;
        if (min < -MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
        u[VEC_Y] = setpoint.Y_throttle;
        rc_saturate_double(&u[VEC_Y], min, max);
        mix_add_input(u[VEC_Y], VEC_Y, mot);
    }

}

static void __run_controller(double* u, double* mot)
{
    // 1) Z Controller
    if (setpoint.en_Z_ctrl) __run_Z_controller();

    // 2) XY Controller
    if (setpoint.en_XY_ctrl) __run_XY_controller();

    // 3) Attitude Controller
    if (setpoint.en_rpy_ctrl) __run_attitude_controller();

    // 4) Attitude Rate Controller
    if (setpoint.en_rpy_rate_ctrl) __run_attitude_rate_controller();

    // 5) Add Throttles to Mixing Matrix
    __add_throttles_to_mixing_matrix(u, mot);
}
#endif /* OFFBOARD_TEST */

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//               Functions Called from Outside "Controller"                  //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


void feedback_controller(double* u, double* mot)
{
    __assign_setpoints_and_enable_loops();

#ifndef OFFBOARD_TEST
    __run_controller(u, mot);
#else
    u = mot;
    mot = u;
#endif
}

int controller_init()
{
    __rpy_init();
    __xyz_init();

    return 0;
}

int controller_reset()
{
    rc_filter_reset(&D_roll_rate_pd);
    rc_filter_reset(&D_roll_rate_i);
    rc_filter_reset(&D_pitch_rate_pd);
    rc_filter_reset(&D_pitch_rate_i);
    rc_filter_reset(&D_yaw_rate_pd);
    rc_filter_reset(&D_yaw_rate_i);
    
    rc_filter_reset(&D_roll_pd);
    rc_filter_reset(&D_roll_i);
    rc_filter_reset(&D_pitch);
    rc_filter_reset(&D_yaw);

    rc_filter_reset(&D_Xdot_pd);
    rc_filter_reset(&D_Xdot_i);
    rc_filter_reset(&D_Ydot_pd);
    rc_filter_reset(&D_Ydot_i);
    rc_filter_reset(&D_Zdot_pd);
    rc_filter_reset(&D_Zdot_i);

    rc_filter_reset(&D_X_pd);
    rc_filter_reset(&D_X_i);
    rc_filter_reset(&D_Y_pd);
    rc_filter_reset(&D_Y_i);
    rc_filter_reset(&D_Z_pd);
    rc_filter_reset(&D_Z_i);

    // prefill filters with zero error (only those with D terms)
    rc_filter_prefill_inputs(&D_roll_rate_pd, 0);
    rc_filter_prefill_inputs(&D_pitch_rate_pd, 0);
    rc_filter_prefill_inputs(&D_yaw_rate_pd, 0);

    rc_filter_prefill_inputs(&D_Xdot_pd, 0);
    rc_filter_prefill_inputs(&D_Ydot_pd, 0);
    rc_filter_prefill_inputs(&D_Zdot_pd, 0);

    return 0;
}

