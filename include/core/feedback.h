/**
 * <feedback.h>
 *
 * @brief   Control module for the vehicle
 *
 * Here lies the heart and soul of the operation. feedback_init(void) pulls
 * in the control constants from settings module and sets up the discrete
 * controllers. From then on out, feedback_march(void) should be called by the
 * IMU interrupt at feedback_hz until the program is shut down.
 * feedback_march(void) will monitor the setpoint which is constantly being
 * changed by setpoint_manager(void).
 *
 * @addtogroup Feedback
 * @{
 */

#ifndef __FEEDBACK__
#define __FEEDBACK__

#include <rc_pilot_defs.h>
#include <stdint.h>  // for uint64_t

/**
 * @brief   Condition of each of the feedback loop's states
 *
 * Reported by the feedback controller. Should only be written to by the
 * feedback controller after initialization.
 * This is the state of the feedback loop. contains most recent values
 */
typedef struct feedback_state_t
{
    int initialized;          ///< set to 1 after feedback_init(void)
    arm_state_t arm_state;    ///< actual arm state as reported by feedback controller
    fail_state_t fail_state;  ///< fail state as reported by the feedback controller
    uint64_t arm_time_ns;     ///< time since boot when controller was armed
    uint64_t loop_index;      ///< increases every time feedback loop runs
    uint64_t last_step_ns;    ///< last time controller has finished a step

    double u[6];  ///< siso controller outputs
    double m[8];  ///< signals sent to motors after mapping
} feedback_state_t;

extern feedback_state_t fstate;

/**
 * @brief   Initial setup of all feedback controllers. Should only be called
 * once on program start.
 *
 * Usage: main.c, line 570
 *
 * @return  0 on success, -1 on failure
 */
int feedback_init(void);

/**
 * @brief   marches feedback controller forward one step
 *
 * This is called AFTER state_estimator_march and actually sends signals to the
 * motors. This is safely callable when Disarmed.
 *
 * Usage: main.c, line 197
 *
 * @return  0 on success, -1 on failure
 */
int feedback_march(void);

/**
 * @brief   This is how outside functions should stop the flight controller.
 *
 * It would be reasonable to set motors to 0 here, but since this
 * function can be called from anywhere that might produce
 * conflicts. Instead the interrupt service routine will do this on
 * the next loop after disarming to maintain timing of pulses to the
 * motors
 *
 * Usage: feedback.c, line 101
 *        feedback.c, line 117
 *        feedback.c, line 123
 *        setpoint_manager.c, line 173
 *        main.c, line 603
 *
 * @return  0 on success, -1 on failure
 */
int feedback_disarm(void);

/**
 * @brief   This is how outside functions should start the flight controller.
 *
 * Usage: setpoint_manager.c, line 179
 *
 * @return  0 on success, -1 on failure
 */
int feedback_arm(void);

/**
 * @brief   Cleanup the feedback controller, freeing memory
 *
 * Usage: main.c, line 652
 *
 * @return  0 on success, -1 on failure
 */
int feedback_cleanup(void);

#endif  // __FEEDBACK__

/**@} end group Feedback */
