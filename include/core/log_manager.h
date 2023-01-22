/**
 * <log_manager.h>
 *
 * @brief   Functions to start, stop, and interact with the log manager
 * thread.
 *
 * @addtogroup LogManager
 * @{
 */

#ifndef __LOG_MANAGER__
#define __LOG_MANAGER__

/**
 * Struct containing all possible values that could be writen to the log. For
 * each log entry you wish to create, fill in an instance of this and pass to
 * add_log_entry(void). You do not need to populate all parts of the struct.
 * Currently feedback.c populates all values and log_manager.c only writes the
 * values enabled in the settings file.
 */
typedef struct log_entry_t
{
    /** @name index, always printed */
    ///@{
    uint64_t loop_index;  // timing
    uint64_t last_step_ns;
    uint64_t imu_time_ns;
    uint64_t bmp_time_ns;
    uint64_t epoch_time_ns;
    ///@}

    /** @name sensors */
    ///@{
    double v_batt;
    double alt_bmp_raw;
    double bmp_temp;
    double gyro_roll;
    double gyro_pitch;
    double gyro_yaw;
    double accel_X;
    double accel_Y;
    double accel_Z;
    double mag_X;
    double mag_Y;
    double mag_Z;
    ///@}

    /** @name PNI RM3100 Magnetometer */
    ///@{
    double rm3100_x;
    double rm3100_y;
    double rm3100_z;
    double cal_rm3100_x;
    double cal_rm3100_y;
    double cal_rm3100_z;
    uint64_t rm3100_time_ns;
    ///@}

    /** @name VL53L1X Laser Range Finder */
    ///@{
    double vl53l1x_dist_m;
    uint64_t vl53l1x_time_ns;
    ///@}

    /** @name state estimate */
    ///@{
    double roll;
    double pitch;
    double yaw;
    double rollDot;
    double pitchDot;
    double yawDot;
    double X;
    double Y;
    double Z;
    double Xdot;
    double Ydot;
    double Zdot;
    double Zddot;
    ///@}

    /*** @name xbee data */
    ///@{
    uint32_t xbee_time;
    uint64_t xbee_time_received_ns;
    float xbee_x;
    float xbee_y;
    float xbee_z;
    float xbee_qw;
    float xbee_qx;
    float xbee_qy;
    float xbee_qz;
    float xbee_roll;
    float xbee_pitch;
    float xbee_yaw;
    ///@}

    /*** @name gps data */
    ///@{
    double gps_lon;        // longitude in degree decimal
    double gps_lat;        // latitude in degree decimal
    double gps_gpsAlt;     // altitude in m (from GPS)
    double gps_ned_x;      // ned x coordinates
    double gps_ned_y;      // ned y coordinates
    double gps_ned_z;      // ned z coordinates
    double gps_spd;        // speed in m/s
    int gps_fix;           // fix type
    uint8_t gps_sat;       // number of satellites
    double gps_headingNc;  // heading (not tilt compensated) in degrees
    double gps_cog;        // course over ground
    double gps_gpsVsi;     // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double gps_hdop;       // horizontal dilution of precision
    double gps_vdop;       // vertical dilution of precision
    uint8_t gps_year;
    uint8_t gps_month;
    uint8_t gps_day;
    uint8_t gps_hour;
    uint8_t gps_minute;
    uint8_t gps_second;
    uint64_t gps_time_received_ns;
    ///@}

    /** @name throttles */
    ///@{
    double X_throttle;
    double Y_throttle;
    double Z_throttle;
    double roll_throttle;
    double pitch_throttle;
    double yaw_throttle;
    ///@}

    /** @name attitude setpoints */
    ///@{
    double sp_roll_dot;
    double sp_pitch_dot;
    double sp_yaw_dot;
    double sp_roll_dot_ff;
    double sp_pitch_dot_ff;
    double sp_yaw_dot_ff;
    double sp_roll;
    double sp_pitch;
    double sp_yaw;
    double sp_roll_ff;
    double sp_pitch_ff;
    ///@}

    /** @name XYZ setpoints */
    ///@{
    double sp_X;
    double sp_Y;
    double sp_Z;
    double sp_Xdot;
    double sp_Ydot;
    double sp_Zdot;
    double sp_Xdot_ff;
    double sp_Ydot_ff;
    double sp_Zdot_ff;
    double sp_Xddot;
    double sp_Yddot;
    double sp_Zddot;
    ///@}

    /** @name orthogonal control outputs */
    ///@{
    double u_roll;
    double u_pitch;
    double u_yaw;
    double u_X;
    double u_Y;
    double u_Z;
    ///@}

    /** @name motor signals */
    ///@{
    double mot_1;
    double mot_2;
    double mot_3;
    double mot_4;
    double mot_5;
    double mot_6;
    double mot_7;
    double mot_8;
    ///@}

    /** @name dsm connection valid */
    ///@{
    int dsm_con;
    ///@}

    /** @name flight mode */
    ///@{
    int flight_mode;
    ///@}

    /** @name imu_isr() Benchmarking Timers */
    ///@{
    uint64_t tIMU, tIMU_END, tSM, tXBEE, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP;
    ///@}

    /** @name Offset Value (and timestamp of offset value) from NTP Server*/
    ///@{
    double ntp_offset_ms;
    uint64_t ntp_time_updated_ns;
    ///@}

    /** @name Realsense Payload data */
    ///@{
    uint64_t rsp_land_time_us;
    float rsp_land_x, rsp_land_y, rsp_land_z;
    uint64_t rsp_pose_time_us;
    float rsp_pose_x, rsp_pose_y, rsp_pose_z, rsp_pose_roll, rsp_pose_pitch, rsp_pose_yaw; 
    ///@}

} log_entry_t;

/**
 * @brief   creates a new csv log file and starts the background thread.
 *
 * @return  0 on success, -1 on failure
 */
int log_manager_init(void);

/**
 * @brief   quickly add new data to local buffer
 *
 * This is called after feedback_march after signals have been sent to
 * the motors.
 *
 * @return  0 on success, -1 on failure
 */
int log_manager_add_new();

/**
 * @brief   Finish writing remaining data to log and close thread.
 *
 * Used in log_manager.c
 *
 * @return  0 on sucess and clean exit, -1 on exit timeout/force close.
 */
int log_manager_cleanup(void);

#endif  // __LOG_MANAGER__

/* @} end group LogManager */
