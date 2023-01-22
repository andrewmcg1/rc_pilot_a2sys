/**
 * @file state_estimator.c
 *
 */

#include <math.h>
#include <stdio.h>

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/led.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/math/quaternion.h>
#include <rc/mpu.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#include <naza_gps.h>
#include <rc_pilot_defs.h>
#include <settings.h>
#include <state_estimator.h>
#include <xbee_receive.h>

#define TWO_PI (M_PI * 2.0)

state_estimate_t state_estimate;  // extern variable in state_estimator.h

// sensor data structs
rc_mpu_data_t mpu_data;
rc_vector_t accel_in = RC_VECTOR_INITIALIZER;
rc_vector_t accel_out = RC_VECTOR_INITIALIZER;
rc_matrix_t rot_matrix = RC_MATRIX_INITIALIZER;
static rc_bmp_data_t bmp_data;

// gyro filter
static rc_filter_t gyro_roll_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t gyro_pitch_lpf = RC_FILTER_INITIALIZER;
static rc_filter_t gyro_yaw_lpf = RC_FILTER_INITIALIZER;

// z filter
static rc_filter_t z_lpf = RC_FILTER_INITIALIZER;

// battery filter
static rc_filter_t batt_lp = RC_FILTER_INITIALIZER;

// altitude filter components
static rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;

static void __batt_init(void)
{
    // init the battery low pass filter
    rc_filter_moving_average(&batt_lp, 20, DT);
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0)
    {
        tmp = settings.v_nominal;
        if (settings.warnings_en)
        {
            fprintf(stderr, "WARNING: ADC read %0.1fV on the barrel jack. Please connect\n", tmp);
            fprintf(stderr, "battery to barrel jack, assuming nominal voltage for now.\n");
        }
    }
    rc_filter_prefill_inputs(&batt_lp, tmp);
    rc_filter_prefill_outputs(&batt_lp, tmp);
    return;
}

static void __batt_march(void)
{
    double tmp = rc_adc_dc_jack();
    if (tmp < 3.0) tmp = settings.v_nominal;
    state_estimate.v_batt_raw = tmp;
    state_estimate.v_batt_lp = rc_filter_march(&batt_lp, tmp);
    return;
}

static void __batt_cleanup(void)
{
    rc_filter_free(&batt_lp);
    return;
}

static void __gyro_init(void)
{
    rc_filter_first_order_lowpass(&gyro_pitch_lpf, DT, settings.gyro_lpf_timeConst_multiplier*DT);
    rc_filter_first_order_lowpass(&gyro_roll_lpf, DT, settings.gyro_lpf_timeConst_multiplier*DT);
    rc_filter_first_order_lowpass(&gyro_yaw_lpf, DT, settings.gyro_lpf_timeConst_multiplier*DT);
    return;
}

static void __gyro_march(void)
{
    state_estimate.roll_dot = rc_filter_march(&gyro_roll_lpf, state_estimate.gyro[0]);
    state_estimate.pitch_dot = rc_filter_march(&gyro_pitch_lpf, state_estimate.gyro[1]);
    state_estimate.yaw_dot = rc_filter_march(&gyro_yaw_lpf, state_estimate.gyro[2]);
    return;
}

static void __gyro_cleanup(void)
{
    rc_filter_free(&gyro_pitch_lpf);
    rc_filter_free(&gyro_roll_lpf);
    rc_filter_free(&gyro_yaw_lpf);
    return;
}

static void __z_init(void)
{
    rc_filter_first_order_lowpass(&z_lpf, DT, 300*DT);
    return;
}

static void __z_march(void)
{
    // state_estimate.Z_ddot = rc_filter_march(&z_lpf, state_estimate.accel_ground_frame[2]);
    state_estimate.Z_ddot = 0;
    return;
}

static void __z_cleanup(void)
{
    rc_filter_free(&z_lpf);
    return;
}

static void __imu_march(void)
{
    static double last_yaw = 0.0;
    static int num_yaw_spins = 0;
    double diff;

    // gyro and accel require converting to NED coordinates
    state_estimate.gyro[0] =  mpu_data.gyro[1] * DEG_TO_RAD;
    state_estimate.gyro[1] =  mpu_data.gyro[0] * DEG_TO_RAD;
    state_estimate.gyro[2] = -mpu_data.gyro[2] * DEG_TO_RAD;

    // quaternion also needs coordinate transform
    state_estimate.quat_imu[0] = mpu_data.dmp_quat[0];   // W
    state_estimate.quat_imu[1] = mpu_data.dmp_quat[2];   // X (i)
    state_estimate.quat_imu[2] = mpu_data.dmp_quat[1];   // Y (j)
    state_estimate.quat_imu[3] = -mpu_data.dmp_quat[3];  // Z (k)

    // normalize it just in case
    rc_quaternion_norm_array(state_estimate.quat_imu);
    // generate tait bryan angles
    rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

    // yaw is more annoying since we have to detect spins
    // also make sign negative since NED coordinates has Z point down
    diff = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI) - last_yaw;
    // detect the crossover point at +-PI and update num yaw spins
    if (diff < -M_PI)
        num_yaw_spins++;
    else if (diff > M_PI)
        num_yaw_spins--;

    // finally the new value can be written
    state_estimate.imu_continuous_yaw = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI);
    last_yaw = state_estimate.imu_continuous_yaw;

    state_estimate.accel[0] = mpu_data.accel[1];
    state_estimate.accel[1] = mpu_data.accel[0];
    state_estimate.accel[2] = -mpu_data.accel[2];
    
    accel_in.d[0] = state_estimate.accel[0];
    accel_in.d[1] = state_estimate.accel[1];
    accel_in.d[2] = state_estimate.accel[2];

    // rotate accel vector
    rc_quaternion_rotate_vector_array(accel_in.d, state_estimate.quat_imu);

    state_estimate.accel_ground_frame[0] = accel_in.d[0];
    state_estimate.accel_ground_frame[1] = accel_in.d[1];
    state_estimate.accel_ground_frame[2] = accel_in.d[2] + GRAVITY;
    return;
}

static void __mag_march(void)
{
    static double last_yaw = 0.0;
    static int num_yaw_spins = 0;

    // don't do anything if mag isn't enabled
    if (!settings.enable_mpu_magnetometer) return;

    // mag require converting to NED coordinates
    state_estimate.mag[0] = mpu_data.mag[1];
    state_estimate.mag[1] = mpu_data.mag[0];
    state_estimate.mag[2] = -mpu_data.mag[2];

    // quaternion also needs coordinate transform
    state_estimate.quat_mag[0] = mpu_data.fused_quat[0];   // W
    state_estimate.quat_mag[1] = mpu_data.fused_quat[2];   // X (i)
    state_estimate.quat_mag[2] = mpu_data.fused_quat[1];   // Y (j)
    state_estimate.quat_mag[3] = -mpu_data.fused_quat[3];  // Z (k)

    // normalize it just in case
    rc_quaternion_norm_array(state_estimate.quat_mag);
    // generate tait bryan angles
    rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

    // heading
    state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
    state_estimate.mag_heading = state_estimate.tb_mag[2];

    // yaw is more annoying since we have to detect spins
    // also make sign negative since NED coordinates has Z point down
    double diff = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI) - last_yaw;
    // detect the crossover point at +-PI and update num yaw spins
    if (diff < -M_PI)
        num_yaw_spins++;
    else if (diff > M_PI)
        num_yaw_spins--;

    // finally the new value can be written
    state_estimate.mag_heading_continuous = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI);
    last_yaw = state_estimate.mag_heading_continuous;
    return;
}

/**
 * @brief      initialize the altitude kalman filter
 *
 * @return     0 on success, -1 on failure
 */
static int __altitude_init(void)
{
    // initialize altitude kalman filter and bmp sensor
    rc_matrix_t F = RC_MATRIX_INITIALIZER;
    rc_matrix_t G = RC_MATRIX_INITIALIZER;
    rc_matrix_t H = RC_MATRIX_INITIALIZER;
    rc_matrix_t Q = RC_MATRIX_INITIALIZER;
    rc_matrix_t R = RC_MATRIX_INITIALIZER;
    rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

    const int Nx = 3;
    const int Ny = 1;
    const int Nu = 1;

    // allocate appropirate memory for system
    rc_matrix_zeros(&F, Nx, Nx);
    rc_matrix_zeros(&G, Nx, Nu);
    rc_matrix_zeros(&H, Ny, Nx);
    rc_matrix_zeros(&Q, Nx, Nx);
    rc_matrix_zeros(&R, Ny, Ny);
    rc_matrix_zeros(&Pi, Nx, Nx);

    // define system -DT; // accel bias
    F.d[0][0] = 1.0;
    F.d[0][1] = DT;
    F.d[0][2] = 0.0;
    F.d[1][0] = 0.0;
    F.d[1][1] = 1.0;
    F.d[1][2] = -DT;  // subtract accel bias
    F.d[2][0] = 0.0;
    F.d[2][1] = 0.0;
    F.d[2][2] = 1.0;  // accel bias state

    G.d[0][0] = 0.5 * DT * DT;
    G.d[0][1] = DT;
    G.d[0][2] = 0.0;

    H.d[0][0] = 1.0;
    H.d[0][1] = 0.0;
    H.d[0][2] = 0.0;

    // covariance matrices
    Q.d[0][0] = 0.000000001;
    Q.d[1][1] = 0.000000001;
    Q.d[2][2] = 0.0001;  // don't want bias to change too quickly
    R.d[0][0] = 1000000.0;

    // initial P, cloned from converged P while running
    Pi.d[0][0] = 1258.69;
    Pi.d[0][1] = 158.6114;
    Pi.d[0][2] = -9.9937;
    Pi.d[1][0] = 158.6114;
    Pi.d[1][1] = 29.9870;
    Pi.d[1][2] = -2.5191;
    Pi.d[2][0] = -9.9937;
    Pi.d[2][1] = -2.5191;
    Pi.d[2][2] = 0.3174;

    // initialize the kalman filter
    if (rc_kalman_alloc_lin(&alt_kf, F, G, H, Q, R, Pi) == -1) return -1;
    rc_matrix_free(&F);
    rc_matrix_free(&G);
    rc_matrix_free(&H);
    rc_matrix_free(&Q);
    rc_matrix_free(&R);
    rc_matrix_free(&Pi);

    // initialize the little LP filter to take out accel noise
    if (rc_filter_first_order_lowpass(&acc_lp, DT, 20 * DT)) return -1;

    // init barometer and read in first data
    if (rc_bmp_read(&bmp_data)) return -1;

    return 0;
}

static void __altitude_march(void)
{
    int i;
    double accel_vec[3];
    static rc_vector_t u = RC_VECTOR_INITIALIZER;
    static rc_vector_t y = RC_VECTOR_INITIALIZER;
    static double global_update;

    // grab raw data (not used if other global update used)
    state_estimate.bmp_pressure_raw = bmp_data.pressure_pa;
    state_estimate.alt_bmp_raw = bmp_data.alt_m;
    state_estimate.bmp_temp = bmp_data.temp_c;

    // Set Global update variable
    global_update = gps_data.lla.alt;

    // make copy of acceleration reading before rotating
    for (i = 0; i < 3; i++) accel_vec[i] = state_estimate.accel[i];

    // rotate accel vector
    rc_quaternion_rotate_vector_array(accel_vec, state_estimate.quat_imu);

    // do first-run filter setup
    if (alt_kf.step == 0)
    {
        rc_vector_zeros(&u, 1);
        rc_vector_zeros(&y, 1);
        alt_kf.x_est.d[0] = -global_update;
        rc_filter_prefill_inputs(&acc_lp, accel_vec[2] + GRAVITY);
        rc_filter_prefill_outputs(&acc_lp, accel_vec[2] + GRAVITY);
    }

    // calculate acceleration and smooth it just a tad
    // put result in u for kalman and flip sign since with altitude, positive
    // is up whereas acceleration in Z points down.
    rc_filter_march(&acc_lp, accel_vec[2] + GRAVITY);
    u.d[0] = acc_lp.newest_output;

    // Use gps for kalman update
    y.d[0] = -global_update;

    rc_kalman_update_lin(&alt_kf, u, y);

    // altitude estimate
    // TODO: rename to something more general (altitude kf from other source than bmp)
    state_estimate.alt_bmp = alt_kf.x_est.d[0];
    state_estimate.alt_bmp_vel = alt_kf.x_est.d[1];
    state_estimate.alt_bmp_accel = alt_kf.x_est.d[2];

    return;
}

/**
 * @brief   Select state estimate values based on flight mode
 */
static void __feedback_select(void)
{
    switch (user_input.flight_mode)
    {
        case AUTONOMOUS:
        case LOITER:
        case LOITER_RSP:
            state_estimate.roll = state_estimate.tb_imu[0];
            state_estimate.pitch = state_estimate.tb_imu[1];
            state_estimate.yaw = state_estimate.tb_imu[2];
            state_estimate.continuous_yaw =
                atan2(2 * (xbeeMsg.qw * xbeeMsg.qz + xbeeMsg.qx * xbeeMsg.qy),
                    1 - 2 * (pow(xbeeMsg.qy, 2) + pow(xbeeMsg.qz, 2)));
            state_estimate.X = xbeeMsg.x;  // TODO: generalize for optitrack and qualisys
            state_estimate.Y = xbeeMsg.y;
            state_estimate.Z = xbeeMsg.z;
            state_estimate.X_dot = xbee_x_dot;
            state_estimate.Y_dot = xbee_y_dot;
            state_estimate.Z_dot = xbee_z_dot;
            break;
        default:
            state_estimate.roll = state_estimate.tb_imu[0];
            state_estimate.pitch = state_estimate.tb_imu[1];
            state_estimate.yaw = state_estimate.tb_imu[2];
            state_estimate.continuous_yaw = state_estimate.mag_heading_continuous;
            state_estimate.X = xbeeMsg.x;
            state_estimate.Y = xbeeMsg.y;
            state_estimate.Z = xbeeMsg.z;
            state_estimate.X_dot = xbee_x_dot;
            state_estimate.Y_dot = xbee_y_dot;
            state_estimate.Z_dot = xbee_z_dot;
            break;
    }
}

static void __altitude_cleanup(void)
{
    rc_kalman_free(&alt_kf);
    rc_filter_free(&acc_lp);
    return;
}

static void __mocap_check_timeout(void)
{
    if (state_estimate.mocap_running)
    {
        uint64_t current_time = rc_nanos_since_epoch();
        // check if mocap data is > 3 steps old
        if ((current_time - state_estimate.mocap_timestamp_ns) > (3 * 1E7))
        {
            state_estimate.mocap_running = 0;
            if (settings.warnings_en)
            {
                fprintf(stderr, "WARNING, MOCAP LOST VISUAL\n");
            }
        }
    }
    return;
}

int state_estimator_init(void)
{
    __batt_init();
    __gyro_init();
    __z_init();
    if (__altitude_init() == -1) return -1;
    if (rc_vector_zeros(&accel_in, 3) == -1) return -1;
    if (rc_vector_zeros(&accel_out, 3) == -1) return -1;
    if (rc_matrix_zeros(&rot_matrix, 3, 3) == -1) return -1;
    state_estimate.initialized = 1;
    return 0;
}

int state_estimator_march(void)
{
    if (state_estimate.initialized == 0)
    {
        fprintf(stderr, "ERROR in state_estimator_march, estimator not initialized\n");
        return -1;
    }

    // populate state_estimate struct one setion at a time, top to bottom
    __batt_march();
    __imu_march();
    __mag_march();
    __altitude_march();
    __gyro_march();
    __z_march();
    __feedback_select();
    __mocap_check_timeout();
    return 0;
}

int state_estimator_jobs_after_feedback(void)
{
    static int bmp_sample_counter = 0;

    // check if we need to sample BMP this loop
    if (bmp_sample_counter >= BMP_RATE_DIV)
    {
        // perform the i2c reads to the sensor, on bad read just try later
        if (rc_bmp_read(&bmp_data)) return -1;
        bmp_sample_counter = 0;
        state_estimate.bmp_time_ns = rc_nanos_since_epoch();
    }
    bmp_sample_counter++;
    return 0;
}

int state_estimator_cleanup(void)
{
    __batt_cleanup();
    __altitude_cleanup();
    __gyro_cleanup();
    __z_cleanup();
    return 0;
}