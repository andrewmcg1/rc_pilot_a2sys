/**
 * @file log_manager.c
 */

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <feedback.h>
#include <log_manager.h>
#include <naza_gps.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <state_estimator.h>
#include <thread_defs.h>
#include <xbee_receive.h>
#include <pni_rm3100.h>
#include <benchmark.h>
#include <ntp_read.h>
#include <realsense_payload_receive.h>
#include <vl53l1x.h>

#define MAX_LOG_FILES 500

static uint64_t num_entries;  ///< number of entries logged so far
static FILE* log_fd;          ///< file descriptor for the log file

static int logging_enabled;  

// Buffer to hold all of the log entries
#define BUF_LEN 120000              // 10 min * 60 sec/min * 200 Hz 
static log_entry_t buffer[BUF_LEN];

static int __write_header(FILE* log_fd)
{
    // always print loop index
    fprintf(log_fd, "loop_index,last_step_ns,imu_time_ns,bmp_time_ns,epoch_time_ns");

    if (settings.log_sensors)
    {
        fprintf(log_fd,
            ",v_batt,alt_bmp_raw,bmp_temp,gyro_roll,gyro_pitch,gyro_yaw,accel_X,accel_Y,accel_Z,mag_X, "
            "mag_Y, mag_Z");
    }

    if (settings.log_state)
    {
        fprintf(log_fd, ",roll,pitch,yaw,rollDot,pitchDot,yawDot,X,Y,Z,Xdot,Ydot,Zdot,Zddot");
    }

    if (settings.log_xbee)
    {
        fprintf(log_fd,
            ",xbee_time,xbee_time_received_ns,xbee_x,xbee_y,xbee_z,xbee_qw,xbee_qx,xbee_qy,xbee_qz,"
            "xbee_roll,xbee_pitch,"
            "xbee_yaw");
    }

    if (settings.log_gps)
    {
        fprintf(log_fd,
            ",gps_lat,gps_lon,gps_gpsAlt,gps_ned_x,gps_ned_y,gps_ned_z,gps_spd,gps_fix,gps_sat,gps_"
            "headingNc,gps_cog,gps_gpsVsi,gps_hdop,gps_vdop,gps_year,gps_month,gps_day,gps_hour,"
            "gps_minute,gps_second,gps_time_received_ns");
    }

    if (settings.log_throttles)
    {
        fprintf(log_fd, ",X_thrt,Y_thrt,Z_thrt,roll_thrt,pitch_thrt,yaw_thrt");
    }

    if (settings.log_attitude_setpoint)
    {
        fprintf(log_fd, ",sp_roll_dot,sp_pitch_dot,sp_yaw_dot,sp_roll_dot_ff,sp_pitch_dot_ff,sp_yaw_dot_ff");
        fprintf(log_fd, ",sp_roll,sp_pitch,sp_yaw,sp_roll_ff,sp_pitch_ff");
    }

    if (settings.log_position_setpoint)
    {
        fprintf(log_fd, ",sp_X,sp_Y,sp_Z,sp_Xdot,sp_Ydot,sp_Zdot");
        fprintf(log_fd, ",sp_Xdot_ff,sp_Ydot_ff,sp_Zdot_ff,sp_Xddot,sp_Yddot,sp_Zddot");
    }

    if (settings.log_control_u)
    {
        fprintf(log_fd, ",u_roll,u_pitch,u_yaw,u_X,u_Y,u_Z");
    }

    if (settings.log_motor_signals && settings.num_rotors == 8)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6,mot_7,mot_8");
    }
    if (settings.log_motor_signals && settings.num_rotors == 6)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6");
    }
    if (settings.log_motor_signals && settings.num_rotors == 4)
    {
        fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4");
    }
    if (settings.log_dsm)
    {
        fprintf(log_fd, ",dsm_con");
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",flight_mode");
    }
    if (settings.log_rm3100)
    {
        fprintf(log_fd, ",rm3100_x,rm3100_y,rm3100_z,cal_rm3100_x,cal_rm3100_y,cal_rm3100_z,rm3100_time_ns");
    }
    if(settings.log_benchmark){
        fprintf(log_fd, ",tIMU,tIMU_END,tSM,tXBEE,tGPS,tPNI,tNAV,tGUI,tCTR,tLOG,tNTP");
    }
    if(settings.log_ntp){
        fprintf(log_fd, ",ntpOffset,ntp_time_updated_ns");
    }
    if(settings.log_realsense_payload){
        fprintf(log_fd, ",rsp_land_time_us,rsp_land_x,rsp_land_y,rsp_land_z,"
                        "rsp_pose_time_us,rsp_pose_x,rsp_pose_y,rsp_pose_z,"
                            "rsp_pose_roll,rsp_pose_pitch,rsp_pose_yaw");
    }
    if (settings.log_vl53l1x)
    {
        fprintf(log_fd, ",vl53l1x_dist_m,vl53l1x_time_ns");
    }

    fprintf(log_fd, "\n");
    return 0;
}

static int __write_log_entry(FILE* log_fd, log_entry_t e)
{
    // always print loop index
    fprintf(log_fd, "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64, 
            e.loop_index, e.last_step_ns, e.imu_time_ns, e.imu_time_ns, e.epoch_time_ns);

    if (settings.log_sensors)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.v_batt,
            e.alt_bmp_raw,e.bmp_temp, e.gyro_roll, e.gyro_pitch, e.gyro_yaw, e.accel_X, e.accel_Y, e.accel_Z,
            e.mag_X, e.mag_Y, e.mag_Z);
    }

    if (settings.log_state)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.roll, e.pitch, e.yaw,
            e.rollDot, e.pitchDot, e.yawDot, e.X, e.Y, e.Z, e.Xdot, e.Ydot, e.Zdot, e.Zddot);
    }

    if (settings.log_xbee)
    {
        fprintf(log_fd, ",%" PRIu32 ",%" PRIu64, e.xbee_time, e.xbee_time_received_ns);
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.xbee_x, e.xbee_y,
            e.xbee_z, e.xbee_qw, e.xbee_qx, e.xbee_qy, e.xbee_qz, e.xbee_roll, e.xbee_pitch,
            e.xbee_yaw);
    }

    if (settings.log_gps)
    {
        fprintf(log_fd,
            ",%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%i,%i,%i,%i",
            e.gps_lat, e.gps_lon, e.gps_gpsAlt, e.gps_ned_x, e.gps_ned_y, e.gps_ned_z, e.gps_spd,
            e.gps_fix, e.gps_sat, e.gps_headingNc, e.gps_cog, e.gps_gpsVsi, e.gps_hdop, e.gps_vdop,
            e.gps_year, e.gps_month, e.gps_day, e.gps_hour, e.gps_minute, e.gps_second);
        fprintf(log_fd, ",%" PRIu64, e.gps_time_received_ns);
    }

    if (settings.log_throttles)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.X_throttle, e.Y_throttle, e.Z_throttle,
            e.roll_throttle, e.pitch_throttle, e.yaw_throttle);
    }

    if (settings.log_attitude_setpoint)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.sp_roll_dot,
            e.sp_pitch_dot, e.sp_yaw_dot, e.sp_roll_dot_ff, e.sp_pitch_dot_ff, e.sp_yaw_dot_ff, e.sp_roll, e.sp_pitch, e.sp_yaw,
            e.sp_roll_ff, e.sp_pitch_ff);
    }

    if (settings.log_position_setpoint)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.sp_X,
            e.sp_Y, e.sp_Z, e.sp_Xdot, e.sp_Ydot, e.sp_Zdot, e.sp_Xdot_ff, e.sp_Ydot_ff, e.sp_Zdot_ff,
            e.sp_Xddot, e.sp_Yddot, e.sp_Zddot);
    }

    if (settings.log_control_u)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.u_roll, e.u_pitch, e.u_yaw, e.u_X,
            e.u_Y, e.u_Z);
    }

    if (settings.log_motor_signals && settings.num_rotors == 8)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.mot_1, e.mot_2, e.mot_3,
            e.mot_4, e.mot_5, e.mot_6, e.mot_7, e.mot_8);
    }
    if (settings.log_motor_signals && settings.num_rotors == 6)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", e.mot_1, e.mot_2, e.mot_3, e.mot_4,
            e.mot_5, e.mot_6);
    }
    if (settings.log_motor_signals && settings.num_rotors == 4)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F", e.mot_1, e.mot_2, e.mot_3, e.mot_4);
    }
    if (settings.log_dsm)
    {
        fprintf(log_fd, ",%i", e.dsm_con);
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",%i", e.flight_mode);
    }

    if (settings.log_rm3100)
    {
        fprintf(log_fd, ",%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%" PRIu64,
                 e.rm3100_x, e.rm3100_y, e.rm3100_z,e.cal_rm3100_x, e.cal_rm3100_y, e.cal_rm3100_z, e.rm3100_time_ns);
    }
    if(settings.log_benchmark){
        fprintf(log_fd, ",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64,
            e.tIMU, e.tIMU_END, e.tSM, e.tXBEE, e.tGPS, e.tPNI, e.tNAV, e.tGUI, e.tCTR, e.tLOG, e.tNTP);
    }
    if(settings.log_ntp){
        fprintf(log_fd, ",%lf,%" PRIu64, e.ntp_offset_ms, e.ntp_time_updated_ns);
    }
        
    if(settings.log_realsense_payload){
        fprintf(log_fd, ",%" PRIu64, e.rsp_land_time_us);
        fprintf(log_fd, ",%.4F,%.4F,%.4F", e.rsp_land_x, e.rsp_land_y, e.rsp_land_z);
        fprintf(log_fd, ",%" PRIu64, e.rsp_pose_time_us);
        fprintf(log_fd, ",%.4F,%.4F,%.4F", e.rsp_pose_x, e.rsp_pose_y, e.rsp_pose_z);
        fprintf(log_fd, ",%.4F,%.4F,%.4F", e.rsp_pose_roll, e.rsp_pose_pitch, e.rsp_pose_yaw);
    }
    if (settings.log_vl53l1x)
    {
        fprintf(log_fd, ",%.4lf,%" PRIu64, e.vl53l1x_dist_m, e.vl53l1x_time_ns);
    }

    fprintf(log_fd, "\n");
    return 0;
}

int log_manager_init()
{
    int i;
    char path[100];
    struct stat st = {0};

    // if the thread if running, stop before starting a new log file
    if (logging_enabled)
    {
        log_manager_cleanup();
    }

    // first make sure the directory exists, make it if not
    if (stat(LOG_DIR, &st) == -1)
    {
        mkdir(LOG_DIR, 0755);
    }

    // search for existing log files to determine the next number in the series
    // TODO: more expressive log name
    for (i = 1; i <= MAX_LOG_FILES + 1; i++)
    {
        memset(&path, 0, sizeof(path));
        sprintf(path, LOG_DIR "%d.csv", i);
        // if file exists, move onto the next index
        if (stat(path, &st) == 0)
            continue;
        else
            break;
    }
    // limit number of log files
    if (i == MAX_LOG_FILES + 1)
    {
        fprintf(stderr, "ERROR: log file limit exceeded\n");
        fprintf(stderr, "delete old log files before continuing\n");
        return -1;
    }
    // create and open new file for writing
    log_fd = fopen(path, "w+");
    if (log_fd == 0)
    {
        printf("ERROR: can't open log file for writing\n");
        return -1;
    }

    // write header
    __write_header(log_fd);

    // start
    logging_enabled = 1;
    num_entries = 0;

    return 0;
}

static log_entry_t __construct_new_entry()
{
    log_entry_t e;

    e.loop_index = fstate.loop_index;
    e.last_step_ns = fstate.last_step_ns;
    e.imu_time_ns = state_estimate.imu_time_ns;
    e.bmp_time_ns = state_estimate.bmp_time_ns;
    e.epoch_time_ns = rc_nanos_since_epoch();

    e.v_batt = state_estimate.v_batt_lp;
    e.alt_bmp_raw = state_estimate.alt_bmp_raw;
    e.bmp_temp = state_estimate.bmp_temp;
    e.gyro_roll = state_estimate.gyro[0];
    e.gyro_pitch = state_estimate.gyro[1];
    e.gyro_yaw = state_estimate.gyro[2];
    e.accel_X = state_estimate.accel[0];
    e.accel_Y = state_estimate.accel[1];
    e.accel_Z = state_estimate.accel[2];
    e.mag_X = state_estimate.mag[0];
    e.mag_Y = state_estimate.mag[1];
    e.mag_Z = state_estimate.mag[2];

    e.roll = state_estimate.roll;
    e.pitch = state_estimate.pitch;
    e.yaw = state_estimate.continuous_yaw;
    e.rollDot = state_estimate.roll_dot;
    e.pitchDot = state_estimate.pitch_dot;
    e.yawDot = state_estimate.yaw_dot;

    e.X = state_estimate.X;
    e.Y = state_estimate.Y;
    e.Z = state_estimate.Z;
    e.Xdot = state_estimate.X_dot;
    e.Ydot = state_estimate.Y_dot;
    e.Zdot = state_estimate.Z_dot;
    e.Zddot = state_estimate.Z_ddot;

    e.xbee_time = xbeeMsg.time;
    e.xbee_time_received_ns = state_estimate.xbee_time_received_ns;
    e.xbee_x = xbeeMsg.x;
    e.xbee_y = xbeeMsg.y;
    e.xbee_z = xbeeMsg.z;
    e.xbee_qw = xbeeMsg.qw;
    e.xbee_qx = xbeeMsg.qx;
    e.xbee_qy = xbeeMsg.qy;
    e.xbee_qz = xbeeMsg.qz;
    // e.xbee_roll = xbeeMsg.roll;
    // e.xbee_pitch = xbeeMsg.pitch;
    // e.xbee_yaw = xbeeMsg.yaw;

    e.gps_lon = gps_data.lla.lon;
    e.gps_lat = gps_data.lla.lat;
    e.gps_gpsAlt = gps_data.lla.alt;
    e.gps_ned_x = gps_data.ned.x;
    e.gps_ned_y = gps_data.ned.y;
    e.gps_ned_z = gps_data.ned.z;
    e.gps_spd = gps_data.spd;
    e.gps_fix = (int)gps_data.fix;
    e.gps_sat = gps_data.sat;
    e.gps_headingNc = gps_data.headingNc;
    e.gps_cog = gps_data.cog;
    e.gps_gpsVsi = gps_data.gpsVsi;
    e.gps_hdop = gps_data.hdop;
    e.gps_vdop = gps_data.vdop;
    e.gps_year = gps_data.year;
    e.gps_month = gps_data.month;
    e.gps_day = gps_data.day;
    e.gps_hour = gps_data.hour;
    e.gps_minute = gps_data.minute;
    e.gps_second = gps_data.second;
    e.gps_time_received_ns = gps_data.gps_data_received_ns;

    e.X_throttle = setpoint.X_throttle;
    e.Y_throttle = setpoint.Y_throttle;
    e.Z_throttle = setpoint.Z_throttle;
    e.roll_throttle = setpoint.roll_throttle;
    e.pitch_throttle = setpoint.pitch_throttle;
    e.yaw_throttle = setpoint.yaw_throttle;

    e.sp_roll_dot = setpoint.roll_dot;
    e.sp_pitch_dot = setpoint.pitch_dot;
    e.sp_yaw_dot = setpoint.yaw_dot;
    e.sp_roll_dot_ff = setpoint.roll_dot_ff;
    e.sp_pitch_dot_ff = setpoint.pitch_dot_ff;
    e.sp_yaw_dot_ff = setpoint.yaw_dot_ff;
    e.sp_roll = setpoint.roll;
    e.sp_pitch = setpoint.pitch;
    e.sp_yaw = setpoint.yaw;
    e.sp_roll_ff = setpoint.roll_ff;
    e.sp_pitch_ff = setpoint.pitch_ff;

    e.sp_X = setpoint.X;
    e.sp_Y = setpoint.Y;
    e.sp_Z = setpoint.Z;
    e.sp_Xdot = setpoint.X_dot;
    e.sp_Ydot = setpoint.Y_dot;
    e.sp_Zdot = setpoint.Z_dot;
    e.sp_Xdot_ff = setpoint.X_dot_ff;
    e.sp_Ydot_ff = setpoint.Y_dot_ff;
    e.sp_Zdot_ff = setpoint.Z_dot_ff;
    e.sp_Xddot = setpoint.X_ddot;
    e.sp_Yddot = setpoint.Y_ddot;
    e.sp_Zddot = setpoint.Z_ddot;

    e.u_roll = fstate.u[VEC_ROLL];
    e.u_pitch = fstate.u[VEC_PITCH];
    e.u_yaw = fstate.u[VEC_YAW];
    e.u_X = fstate.u[VEC_Y];
    e.u_Y = fstate.u[VEC_X];
    e.u_Z = fstate.u[VEC_Z];

    e.mot_1 = fstate.m[0];
    e.mot_2 = fstate.m[1];
    e.mot_3 = fstate.m[2];
    e.mot_4 = fstate.m[3];
    e.mot_5 = fstate.m[4];
    e.mot_6 = fstate.m[5];
    e.mot_7 = fstate.m[6];
    e.mot_8 = fstate.m[7];

    e.dsm_con = user_input.input_active;

    e.flight_mode = user_input.flight_mode;

    e.rm3100_x = rm3100_data_extern.mag[0];
    e.rm3100_y = rm3100_data_extern.mag[1];
    e.rm3100_z = rm3100_data_extern.mag[2];
    e.cal_rm3100_x = rm3100_data_extern.cal_mag[0];
    e.cal_rm3100_y = rm3100_data_extern.cal_mag[1];
    e.cal_rm3100_z = rm3100_data_extern.cal_mag[2];
    e.rm3100_time_ns = rm3100_data_extern.timestamp;
    
    e.vl53l1x_dist_m = vl53l1_data_extern.distance_m;
    e.vl53l1x_time_ns = vl53l1_data_extern.timestamp;

    e.tIMU = benchmark_timers.tIMU;
    e.tIMU_END = benchmark_timers.tIMU_END;
    e.tSM = benchmark_timers.tSM;
    e.tXBEE = benchmark_timers.tXBEE;
    e.tGPS = benchmark_timers.tGPS;
    e.tPNI = benchmark_timers.tPNI;
    e.tNAV = benchmark_timers.tNAV;
    e.tGUI = benchmark_timers.tGUI;
    e.tCTR = benchmark_timers.tCTR;
    e.tLOG = benchmark_timers.tLOG;
    e.tNTP = benchmark_timers.tNTP;

    e.ntp_offset_ms = ntp_data.ntp_offset_ms;
    e.ntp_time_updated_ns = ntp_data.ntp_time_updated_ns;
    
    e.rsp_land_time_us = rspLandingCommandMsg.time_us;
    e.rsp_land_x       = rspLandingCommandMsg.x;
    e.rsp_land_y       = rspLandingCommandMsg.y;
    e.rsp_land_z       = rspLandingCommandMsg.z;
    e.rsp_pose_time_us = rspPoseUpdateMsg.time_us;
    e.rsp_pose_x       = rspPoseUpdateMsg.x;
    e.rsp_pose_y       = rspPoseUpdateMsg.y;
    e.rsp_pose_z       = rspPoseUpdateMsg.z;
    e.rsp_pose_roll    = rspPoseUpdateMsg.roll;
    e.rsp_pose_pitch   = rspPoseUpdateMsg.pitch;
    e.rsp_pose_yaw     = rspPoseUpdateMsg.yaw;

    return e;
}

int log_manager_add_new()
{
    if (!logging_enabled)
    {
        if (settings.log_only_while_armed)
        {
            return 0;
        }   
        else 
        {
            fprintf(stderr, "ERROR: trying to log entry while logger isn't running\n");
            return -1;
        }
    }
    
    // Fill up the buffer in ring-style to save the last 10 minutes always
    buffer[num_entries % BUF_LEN] = __construct_new_entry();
    num_entries++;

    return 0;
}

int log_manager_cleanup()
{
    // just return if not logging
    if (logging_enabled == 0) return 0;

    // Actually write the logfile to disk (handling cycling around ring buffer)
    uint64_t start_idx = 0;
    if (num_entries > BUF_LEN) {
        start_idx = num_entries-BUF_LEN;
    }
    for (uint64_t i=start_idx; i<num_entries; i++) {
        __write_log_entry(log_fd, buffer[i % BUF_LEN]);
    }

    // Clean Up
    logging_enabled = 0;
    num_entries = 0;
    fflush(log_fd);
    fclose(log_fd);

    return 0;
}
