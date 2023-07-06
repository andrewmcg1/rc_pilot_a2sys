/**
 * <setpoint_manager.h>
 *
 * @brief   Guidance module for the vehicle
 *
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint. currently it contains very
 * simply logic and runs very quickly which is why it's okay to run
 * in the feedback ISR right before the feedback controller. In the
 * future this is where go-home and other higher level autonomy will
 * live.
 *
 * This serves to allow the feedback controller to be as simple and
 * clean as possible by putting all high-level manipulation of the
 * setpoints here. Then feedback-controller only needs to march the
 * filters and zero them out when arming or enabling controllers
 *
 * @addtogroup SetpointManager
 * @{
 *
 */

#ifndef __SETPOINT_MANAGER__
#define __SETPOINT_MANAGER__

#include <rc_pilot_defs.h>
#include <stdbool.h>

 /**
  * Setpoint for the feedback controllers.
  *
  * This is written by setpoint_manager and primarily read in by fly_controller.
  * May also be read by printf_manager and log_manager for telemetry.
  */
typedef struct setpoint_t
{
    /** @name general */
    ///< @{
    int initialized;  ///< Incdicates setpoint manager initialization. 1 - Initialized, 0 - Not initialized.
    int en_6dof;      ///< 6DOF toggle. 1 - Initialized, 0 - Not initialized.
    ///< @}

    /**
     * @name Waypoint Management
     *
     * TODO: consider moving path management variables into the path_t struct
     */
     ///< @{
    uint64_t time_auto_set;     ///< time autonomous mode is set and armed
    uint64_t waypoint_time;     ///< Current time. Used to compare to waypoint times.
    int auto_armed_set;         ///< flag to manage time auto + armed is set
    uint64_t cur_waypoint_num;  ///< current waypoint to control to
    bool following_rsp_cmd;     ///< flag to keep track of rsp landing
    ///< @}

    /** @name Mixing Matrix Inputs
     * Normalized Forces and Torques inputs to mixing matrix (can be directly set if desired)
     */
     ///< @{
    double Z_throttle;      ///< Throttle in the Z direction. Positive down.
    double X_throttle;      ///< Throttle in the X direction. Only used when 6dof is enabled. Positive forward.
    double Y_throttle;      ///< Throttle in the Y direction. Only used when 6dof is enabled. Positive right.
    double roll_throttle;   ///< desired normalized torque about x (roll)
    double pitch_throttle;  ///< desired normalized torque about y (pitch)
    double yaw_throttle;    ///< desired normalized torque about z (yaw)
    ///< @}

    /** @name Attitude rate setpoint */
    ///< @{
    int en_rpy_rate_ctrl;    ///< enable the roll, pitch, and yaw rate controllers
    double roll_dot;         ///< roll angle rate (rad/s)
    double pitch_dot;        ///< pitch angle rate (rad/s)
    double yaw_dot;          ///< yaw angle rate (rad/s)
    double roll_dot_ff;      ///< feedforward roll rate (rad/s)
    double pitch_dot_ff;     ///< feedforward pitch rate (rad/s)
    double yaw_dot_ff;       ///< feedforward yaw rate (rad/s)

    ///< @}

    /** @name Attitude setpoint */
    ///< @{
    int en_rpy_ctrl;          ///< enable the roll pitch yaw controllers
    double roll;              ///< roll angle (positive tip right) (rad)
    double pitch;             ///< pitch angle (positive tip back) (rad)
    double yaw;               ///< glabal yaw angle, positive left
    double roll_ff;           ///< feedforward roll angle (rad)
    double pitch_ff;          ///< feedforward pitch angle (rad)
    ///< @}

    /** @name Acceleration setpoint */
    ///< @{
    double X_ddot;
    double Y_ddot;
    double Z_ddot;
    ///< @}

    /** @name Altitude */
    ///< @{
    int en_Z_ctrl;   ///< enable altitude feedback.
    double Z;        ///< vertical distance from where controller was armed
    double Z_dot;    ///< vertical velocity m/s, remember Z points down
    double Z_dot_ff; ///< feedforward vertical velocity (m/s)
    ///< @}

    /** @name horizontal velocity setpoint */
    ///< @{
    double X_dot;       ///< x velocity (m/s)
    double Y_dot;       ///< y velocity (m/s)
    double X_dot_ff;    ///< feedforward x velocity (m/s)
    double Y_dot_ff;    ///< feedforward y velocity (m/s)
    ///< @}

    /** @name horizontal position setpoint */
    ///< @{
    int en_XY_ctrl;
    double X;
    double Y;
    ///<@}

} setpoint_t;

extern setpoint_t setpoint;

/**
 * @brief   Initializes the setpoint manager.
 *
 * Usage: main.c, line 350
 *
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief   Updates the setpoint manager.
 *
 * Should be called before feedback loop.
 *
 * Usage: main.c, line 193
 *
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief   cleans up the setpoint manager.
 *
 *  Not really necessary but here for completeness
 *
 * Usage: main.c, line 654
 *
 * @return  0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);

/**
 * @brief   Externally order setpoint manager to follow new path from path_file
 *
 * Usage: state_machine.c, line 105
 *        state_machine.c, line 162
 *        state_machine.c, line 217
 *
 * @param[in] path_file Filepath
 *
 * @return  0 on success, -1 if unsuccessful
 */
int set_new_path(const char* path_file);

/**
 * @brief   Update yaw value based on yaw_dot and user stick
 *
 * Usage: controller.c, line 179
 *        controller.c, line 209
 *        controller.c, line 251
 *        controller.c, line 327
 *        controller.c, line 346
 *        setpoint_manager.c, line 403
 */
void setpoint_update_yaw(void);

/**
 * @brief   Calculates angle rate of drone movement, using the betaflight rate format
 *
 * Usage: controller.c, line 304
 *        controller.c, line 305
 *        controller.c, line 306
 */
double betaflight_acro_rate(double rcCommand, double rcRate, double superRate, double expo);

/**
 * @brief   Update Z value based on z_dot and user stick
 *
 * Usage: controller.c, line 208
 *        setpoint_manager.c, line 402
 */
void setpoint_update_Z(void);

/**
 * @brief   Update the x and y position setpoints
 *
 * X and Y position setpoints are updated based on the X_dot and Y_dot
 * values generated from the user input.
 *
 * Namely, X_dot = X_stick * X_dot_max (similarly for Y_dot)
 *
 * Usage: setpoint_manager.c, line 401
 */
void setpoint_update_XY_pos(void);

/**
 * @brief Update the setpoint from the next waytpoint
 *
 * Usage: controller.c, line 221
 *        controller.c, line 285
 */
void setpoint_update_setpoint_from_waypoint();

/**
 * @brief Update the position setpoint so its equal to the current position estimate
 *
 * Used when transitioning to ALT_HOLD or LOITER_RSP
 *
 * Usage: controller.c, line 197
 *        controller.c, line 270
 */
void setpoint_update_XYZ_bumpless();

/**
 * @brief Update the position and yaw setpoints based on stick inputs
 *
 * Usage: controller.c, line 290
 */
void setpoint_update_loiter();

/**
 * @brief Returns true if we just transitioned flight modes OR just armed into one.
 *
 * Usage: controller.c, line 193
 *        controller.c, line 26
 */
bool just_transitioned_flight_mode();

/**
 * @brief Plan a 2-stage landing trajectory based on the RSP landing command
 *
 * Usage: main.c, line 144
 */
void plan_rsp_landing();

#endif /* __SETPOINT_MANAGER__ */

/* @} end group SetpointManager */