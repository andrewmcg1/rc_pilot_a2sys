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
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t
{
    /** @name general */
    ///< @{
    int initialized;  ///< set to 1 once setpoint manager has initialized
    int en_6dof;      ///< enable 6DOF control features
    ///< @}

    /**
     * @name Waypoint Management
     *
     * TODO: consider moving path management variables into the path_t struct
     */
    ///< @{
    uint64_t time_auto_set;     ///< time autonomous mode is set and armed
    uint64_t waypoint_time;     ///< current time to compare to waypoint times
    int auto_armed_set;         ///< flag to manage time auto + armed is set
    uint64_t cur_waypoint_num;  ///< current waypoint to control to
    bool following_rsp_cmd;     ///< flag to keep track of rsp landing
    ///< @}

    /** @name Mixing Matrix Inputs - Normalized Forces and Torques
     * inputs to mixing matrix (can be directly set if desired)
     */
    ///< @{
    double Z_throttle;      ///< positive down
    double X_throttle;      ///< only used when 6dof is enabled, positive forward
    double Y_throttle;      ///< only used when 6dof is enabled, positive right
    double roll_throttle;   ///< desired normalized torque about x (roll)
    double pitch_throttle;  ///< desired normalized torque about y (pitch)
    double yaw_throttle;    ///< desired normalized torque about z (yaw)
    ///< @}

    /** @name attitude rate setpoint */
    ///< @{
    int en_rpy_rate_ctrl;     ///< enable the roll pitch yaw rate controllers
    double roll_dot;         ///< roll angle rate (rad/s)
    double pitch_dot;        ///< pitch angle rate (rad/s)
    double yaw_dot;          ///< yaw angle rate (rad/s)
    double roll_dot_ff;      ///< feedforward roll rate (rad/s)
    double pitch_dot_ff;     ///< feedforward pitch rate (rad/s)
    double yaw_dot_ff;       ///< feedforward yaw rate (rad/s)
    
    ///< @}

    /** @name attitude setpoint */
    ///< @{
    int en_rpy_ctrl;          ///< enable the roll pitch yaw controllers
    double roll;              ///< roll angle (positive tip right) (rad)
    double pitch;             ///< pitch angle (positive tip back) (rad)
    double yaw;               ///< glabal yaw angle, positive left
    double roll_ff;           ///< feedforward roll angle (rad)
    double pitch_ff;          ///< feedforward pitch angle (rad)
    ///< @}

    /** @name acceleration setpoint */
    ///< @{
    double X_ddot;
    double Y_ddot;
    double Z_ddot;
    ///< @}

    /** @name altitude */
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
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief   updates the setpoint manager, call this before feedback loop
 *
 * @return  0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief   cleans up the setpoint manager, not really necessary but here for
 *             completeness
 *
 * @return  0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);

/**
 * @brief   Externally order setpoint manager to follow new path from path_file
 *
 * @return  0 on success, -1 if unsuccessful
 */
int set_new_path(const char* path_file);

/**
 * @brief   Update yaw value based on yaw_dot and user stick
 */
void setpoint_update_yaw(void);

/**
 * @brief   Update Z value based on z_dot and user stick
 */
void setpoint_update_Z(void);

/**
 * @brief   Update the x and y position setpoints
 *
 * X and Y position setpoints are updated based on the X_dot and Y_dot
 * values generated from the user input.  Namely, X_dot = X_stick * X_dot_max (similarly for Y_dot)
 */
void setpoint_update_XY_pos(void);

/**
 * @brief Update the setpoint form the next waytpoint
 */
void setpoint_update_setpoint_from_waypoint();

/**
 * @brief Update the position setpoint so its equal to the current position estimate
 */
void setpoint_update_XYZ_bumpless();

/**
 * @brief Update the position and yaw setpoints based on stick inputs
 */
void setpoint_update_loiter();

/**
 * @brief Returns true if we just transitioned flight modes OR just armed into one.
 */
bool just_transitioned_flight_mode();

/**
 * @brief Plan a 2-stage landing trajectory based on the RSP landing command
 */
void plan_rsp_landing();

#endif /* __SETPOINT_MANAGER__ */

/* @} end group SetpointManager */