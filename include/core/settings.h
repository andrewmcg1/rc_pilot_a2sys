/**
 * <settings.h>
 *
 * @brief   Functions to read the json settings file
 *
 * @addtogroup Settings
 * @{
 */

#ifndef __SETTINGS__
#define __SETTINGS__

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include <flight_mode.h>
#include <input_manager.h>
#include <mix.h>
#include <rc_pilot_defs.h>
#include <thrust_map.h>

 /**
  * Configuration settings read from the json settings file and passed to most
  * threads as they initialize.
  */
typedef struct settings_t
{
    /** @name File details */
    char name[128];  ///< string declaring the name of the settings file
    ///@}

    /**@name warings */
    ///@{
    int warnings_en;
    ///@}

    /** @name physical parameters */
    ///@{
    int num_rotors;
    rotor_layout_t layout;
    int dof;
    thrust_map_t thrust_map;
    double v_nominal;
    int enable_mpu_magnetometer;  // we suggest leaving as 0 (mag OFF)
    ///@}

    //True if you want to automatically enter OPEN_LOOP_DESCENT if using a mode that
    //requires MOCAP and mocap has been unavailable for 'mocap_dropout_timeout_ms' ms
    //OPEN_LOOP_DESCENT commands 0 roll, 0 pitch, and throttle of 'dropout_z_throttle'
    //One can exit this mode by switching to a controller mode that doesn't require MOCAP
    int enable_mocap_dropout_emergency_land;
    double mocap_dropout_timeout_ms;

    //-0.6 is close to hovering. Make the value closer to zero (less negative) if you want to descend faster. 
    //This parameter will also depend on the weight of the vehicle.
    double dropout_z_throttle;

    double hover_throttle;
    double output_modifier;

    // Amount of time (in seconds) between each RM3100 read
    double rm3100_time_between_meas_s;
    double vl53l1x_time_between_meas_s;
    uint16_t vl53l1x_timing_budget_ms;

    // 
    int apply_rm3100_calibration;
    char rm3100_calibration_filename[200];
    int rm3100_CCR_value;
    int rm3100_i2c_addr;

    /** @name flight modes */
    ///@{
    int num_dsm_modes;
    flight_mode_t flight_mode_1;
    flight_mode_t flight_mode_2;
    flight_mode_t flight_mode_3;
    ///@}

    /** @name dsm radio config */
    ///@{
    int dsm_thr_ch;
    int dsm_thr_pol;
    int dsm_roll_ch;
    int dsm_roll_pol;
    int dsm_pitch_ch;
    int dsm_pitch_pol;
    int dsm_yaw_ch;
    int dsm_yaw_pol;
    int dsm_mode_ch;
    int dsm_mode_pol;
    dsm_kill_mode_t dsm_kill_mode;
    int dsm_kill_ch;
    int dsm_kill_pol;
    ///@}

    /** @name printf settings */
    ///@{
    int printf_arm;
    int printf_altitude;
    int printf_battery;
    int printf_rpy;
    int printf_sticks;
    int printf_setpoint;
    int printf_u;
    int printf_motors;
    int printf_mode;
    int printf_xbee;
    int printf_xbee_velocities;
    int printf_tracking;
    int printf_gps;
    int printf_magnetom;
    int printf_rm3100;
    int printf_vl53l1x;

    ///@}

    /** @name log settings */
    ///@{
    int enable_logging;
    int log_only_while_armed;
    int log_sensors;
    int log_state;
    int log_xbee;
    int log_gps;
    int log_attitude_setpoint;
    int log_position_setpoint;
    int log_control_u;
    int log_motor_signals;
    int log_throttles;
    int log_dsm;
    int log_flight_mode;
    int log_rm3100;
    int log_benchmark;
    int log_ntp;
    int log_realsense_payload;
    int log_vl53l1x;
    ///@}

    /** @name mavlink stuff */
    ///@{
    char dest_ip[24];
    uint8_t my_sys_id;
    uint16_t mav_port;
    ///@}

    /** @name gyroscope low pass filter time constant multiplier */
    ///@{
    double gyro_lpf_timeConst_multiplier;
    ///@}

    /** @name feedback controllers */
    ///@{
    rc_filter_t roll_rate_controller_pd;
    rc_filter_t roll_rate_controller_i;
    rc_filter_t pitch_rate_controller_pd;
    rc_filter_t pitch_rate_controller_i;
    rc_filter_t yaw_rate_controller_pd;
    rc_filter_t yaw_rate_controller_i;
    rc_filter_t roll_controller_pd;
    rc_filter_t roll_controller_i;
    rc_filter_t pitch_controller;
    rc_filter_t yaw_controller;
    rc_filter_t altitude_rate_controller_pd;
    rc_filter_t altitude_rate_controller_i;
    rc_filter_t altitude_controller_pd;
    rc_filter_t altitude_controller_i;
    rc_filter_t horiz_vel_ctrl_pd;
    rc_filter_t horiz_vel_ctrl_i;
    rc_filter_t horiz_pos_ctrl_pd;
    rc_filter_t horiz_pos_ctrl_i;

    ///@}

    /** @name dsm connection */
    ///@{
    int dsm_timeout_ms;
    ///@}

    /** @name enable waypoint linear interpolation */
    ///@{
    int enable_wp_linear_interpolation;
    ///@}

    /** @name waypoint folder name and path*/
    ///@{
    char wp_folder[200];
    ///@}

    /** @name waypoint filenames for takeoff, guided, and landing */
    ///@{
    char wp_takeoff_filename[100];
    char wp_guided_filename[100];
    char wp_loiter_filename[100];
    char wp_landing_filename[100];
    ///@}

    /** @name serial port the Xbee is connected to */
    ///@{
    char xbee_serial_port[50];
    int xbee_packet_version;
    ///@}

    /** @name serial port that the delta arm is connected too*/
    ///@{
    int delta_arm_bus;
    int delta_arm_enable;
    ///@}

    /** @name Realsense Payload Settings */
    ///@{
    char realsense_payload_serial_port[50];
    ///@}
} settings_t;

/**
 * settings are external, so just include this header and read from it
 */
extern settings_t settings;

/**
 * @brief   Populates the settings and controller structs with the settings file.
 *
 * Usage: main.c, line 287
 *
 * @return  0 on success, -1 on failure
 */
int settings_load_from_file(const char* path);

/**
 * @brief   Only used in debug mode. Prints settings to console
 *
 * Usage: settings.c, line 654
 *
 * @return  0 on success, -1 on failure
 */
int settings_print(void);

#endif /* __SETTINGS__ */

/* @} end group Settings */