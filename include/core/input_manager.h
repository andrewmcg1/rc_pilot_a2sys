/**
 * <input_manager.h>
 *
 * @brief   A DSM radio input manager.
 *
 * Contains Functions to start and stop the input manager thread which is the translation
 * beween control inputs from DSM to the user_input struct which is read by the
 * setpoint manager.
 *
 * TODO: Allow other inputs such as mavlink
 *
 * @addtogroup InputManager
 * @{
 */

#ifndef __INPUT_MANAGER__
#define __INPUT_MANAGER__

#include <feedback.h>  // only for arm_state_t  TODO: get rid of cross-polination
#include <flight_mode.h>

 /**
  * @brief   Determines how the dsm radio indicates an arm/disarm kill switch
  */
typedef enum dsm_kill_mode_t {
    /**
     * A dedicated channel is used as a kill switch.
     *
     * Carefully set the dsm_kill_ch and dsm_kill_pol channel and polarity settings.
     */
    DSM_KILL_DEDICATED_SWITCH,
    /**
     *
     * Negative throttle is used as the kill switch.
     *
     * This is the preferred method because it frees up a channel for other use.
     * When using this mode, dsm_kill_ch and dsm_kill_pol are ignored.
     *
     * Some radios, such as Spektrum DXe have an ARM/DISARM switch which
     * forces the throttle channel down below normal range to disarm.
     */
    DSM_KILL_NEGATIVE_THROTTLE
} dsm_kill_mode_t;

/**
 * @brief   Current user input to the system
 *
 * Represents current inputs by the user.
 *
 * This is populated by the input_manager thread.
 * THe input_manager thread decides to read from mavlink or DSM.
 */
typedef struct user_input_t {
    int initialized;                 ///< Set to 1 after input manager is initialized
    flight_mode_t flight_mode;       ///< User determined flight_mode
    flight_mode_t prev_flight_mode;  ///< Flight mode from the previous loop
    int input_active;                ///< Indicates if there has been an input
    int kill_switch;                 ///< For printing
    arm_state_t requested_arm_mode;  ///< Set to ARMED after arming sequence is entered.

    // All sticks scaled from -1 to 1
    double thr_stick;    ///< Throttle stick. Positive forward.
    double yaw_stick;    ///< Yaw stick. Positive to the right. CW yaw.
    double roll_stick;   ///< Roll input. Positive to the right.
    double pitch_stick;  ///< Pitch stick. Positive forward.
} user_input_t;

extern user_input_t user_input;

/**
 * @brief   Starts an input manager thread.
 *
 * Watch for new DSM data and translate into local user mode.
 *
 * Usage: main.c, line 376
 *
 * @return  0 on success, -1 on failure
 */
int input_manager_init(void);

/**
 * @brief   Waits for the input manager thread to exit
 *
 * This should only be called after the program flow state is set to EXITING.
 *
 * The flow state being set to EXITING is the only thing that will allow the thread to exit safely.
 *
 * Usage: main.c, line 653
 *
 * @return  0 on clean exit, -1 if exit timed out.
 */
int input_manager_cleanup(void);

/**
 * @brief Applies a dead zone to an input stick in.
 *
 * in must range from -1 to 1.
 *
 * The deadzone is centered around 0.
 *
 * Zone must range from 0 to 1.
 *
 * Usage: setpoint_manager.c, line 81
 *        setpoint_manager.c, line 399
 *        setpoint_manager.c, line 400
 *        input_manager.c, line 106
 *
 * @param[in] in Normalized input value
 * @param[in] zone Normalized size of deadzone
 *
 * @return  in value with applied deadzone
 */
double deadzone(double in, double zone);

#endif /* __INPUT_MANAGER__ */

/**@}end group InputMaganger */