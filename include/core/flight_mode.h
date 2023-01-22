/**
 * <flight_mode.h>
 *
 * @brief   Flight mode definitions
 *
 * @addtogroup FlightModes
 * @{
 */

#ifndef __FLIGHT_MODE__
#define __FLIGHT_MODE__

#include <stdbool.h>


/**
 * This is how the user interacts with the setpoint manager.
 */
typedef enum flight_mode_t
{
    /**
     * test_bench mode does no feedback at all, it takes the raw user inputs
     * and directly outputs to the motors. This could technically fly but
     * would not be easy! Designed for confirming mixing matrix and motors
     * are working. maps Z,Roll,Pitch,Yaw
     */
    TEST_BENCH_4DOF,
    /**
     * test_bench mode does no feedback at all, it takes the raw user inputs
     * and directly outputs to the motors. This could technically fly but
     * would not be easy! Designed for confirming mixing matrix and motors
     * are working. maps X,Y,Z,Yaw
     */
    TEST_BENCH_6DOF,
    /**
     * RP_TEST_DIRECT_THROTTLE is intended to test the roll and pitch
     * controllers while teathered to the pvc structer.  Direct throttle is
     * passed through (i.e. no automatic altitude control)
     */
    RP_TEST_DIRECT_THROTTLE,
    /**
     * user inputs translate directly to the throttle, roll, pitch, & yaw
     * setpoints. No altitude feedback control. On 6DOF platforms the X & Y
     * thrust directions are left at 0.
     */
    MANUAL,
    /**
     * like DIRECT_THROTTLE for roll/pitch/yaw but feedback is performed to
     * hold altitude setpoint which is them moved up and down steadily based
     * on user input.
     * Can be used to tune Z controller.
     */
    ALT_HOLD,
    /**
     * Reads waypoint file and controls to location at time
     */
    AUTONOMOUS,
    /**
     * Commands 0 roll, 0 pitch, and a fixed throttle to hover or slowely descend
     * Useful as an emergency mode if MOCAP drops out for too long
     */
    OPEN_LOOP_DESCENT,
    /**
     * Control sticks modify the position setpoint by acting like velocity commands.
     * Sticks at neutral points will hold the current setpoint. 
     * Like AUTONOMOUS but inputs come from the control sticks.
     */
    LOITER,
    /**
     * LOITER + Landing from Realsense Payload (RSP) Landing Command.
     * Sticks control the current setpoint. Once a RSP landing command is received, the sticks 
     * no longer update the setpoints and instead a 2-part trajectory is followed:
     * 1) Constant Z, XY translation to arrive above desired landing point, 
     * 2) Constant XY, Z translation until landed
     */
    LOITER_RSP,
    /**
     * Attitude sticks command attitude rate (roll, pitch, yaw)
     * Throttle stick directly controls throttle
     * Can be used for fpv drone racing and also attitude rate tuning in the pvc rig
     */
    ACRO,
    /**
     * MANUAL attitude stick control (roll & pitch angles, yaw rate)
     * Throttle stick directly controls desired Z acceleration
     * Used for testing/tuning throttle controller
     */
    TEST_BENCH_DIRECT_Z_ACC,
    /**
     * MANUAL attitude stick control
     * Throttle stick directly controls desired Z acceleration
     * Used for testing/tuning z_dot controller
     */
    TEST_BENCH_DIRECT_Z_VEL
} flight_mode_t;

/** This function is balically a list of the enums in 'flight_mode_t'
 * that require Motion Capture (MOCAP) to function properly. 
 */
bool mode_needs_mocap(flight_mode_t mode);

#endif /* __FLIGHT_MODE__ */

/**@}end group Flight Modes */