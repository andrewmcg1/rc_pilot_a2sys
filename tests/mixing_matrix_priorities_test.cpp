#include <boost/test/unit_test.hpp>

extern "C"
{
    // Just include everything that main.c does...
    #include <getopt.h>
    #include <stdio.h>
    #include <unistd.h>
    #include <stdlib.h>

    #include <rc/adc.h>
    #include <rc/bmp.h>
    #include <rc/button.h>
    #include <rc/cpu.h>
    #include <rc/dsm.h>
    #include <rc/led.h>
    #include <rc/mpu.h>
    #include <rc/servo.h>
    #include <rc/start_stop.h>
    #include <rc/time.h>
    #include <rc/math.h>

    #include <input_manager.h>
    #include <log_manager.h>
    #include <mix.h>
    #include <naza_gps.h>
    #include <path.h>
    #include <printf_manager.h>
    #include <setpoint_manager.h>
    #include <settings.h>  // contains extern settings variable
    #include <state_estimator.h>
    #include <state_machine.h>
    #include <thrust_map.h>
    #include <xbee_receive.h>
    #include <pni_rm3100.h>
    #include <benchmark.h>
    #include <ntp_read.h>
    #include <realsense_payload_receive.h>
}

void add_throttles_to_mixing_matrix(double* u, double* mot);
void add_throttles_to_mixing_matrix_v2(double* u, double* mot);

BOOST_AUTO_TEST_SUITE(Test_Mixing_Matrix_Priorities)

BOOST_AUTO_TEST_CASE(test_mixing_matrix_io)
{
    char* settings_file_path = "settings/m330_quad_4s_coop_payload.json";


    float x = 0;
    double u[6], mot[8];
    for (int i = 0; i < 8; i++) mot[i] = 0.0;
    for (int i = 0; i < 6; i++) u[i] = 0.0;

    // // 1) Setup
    if (settings_load_from_file(settings_file_path) < 0)
    {
        fprintf(stderr, "ERROR: failed to load settings %s\n",settings_file_path);
    }
    printf("initializing thrust map\n");
    if (thrust_map_init(settings.thrust_map) < 0)
    {
        fprintf(stderr, "ERROR: failed to initialize thrust map\n");
    }
    printf("initializing mixing matrix\n");
    if (mix_init(settings.layout) < 0)
    {
        fprintf(stderr, "ERROR: failed to initialize mixing matrix\n");
    }
    printf("initializing feedback controller\n");
    if (feedback_init() < 0)
    {
        fprintf(stderr, "ERROR: failed to init feedback controller\n");
    }

    // 2) Assign Input
    setpoint.roll_throttle =  0;
    setpoint.pitch_throttle = 0;
    setpoint.yaw_throttle =   0;
    setpoint.X_throttle =   0.0;
    setpoint.Y_throttle =   0.0;
    setpoint.Z_throttle =    -1;

    printf("Desired  Throttles:");
    printf("fx: %2.3f, fy: %2.3f, fz: %2.3f, ",setpoint.X_throttle,setpoint.Y_throttle,setpoint.Z_throttle);
    printf("tx: %2.3f, ty: %2.3f, tz: %2.3f \n",setpoint.roll_throttle,setpoint.pitch_throttle,setpoint.yaw_throttle);

    // 3) Run Mixing Matrix Function
    add_throttles_to_mixing_matrix_v2(u, mot);

    // 4) Display Results
    printf("Achieved Throttles:");
    printf("fx: %2.3f, fy: %2.3f, fz: %2.3f, ",u[0],u[1],u[2]);
    printf("tx: %2.3f, ty: %2.3f, tz: %2.3f \n",u[3],u[4],u[5]);

    printf("Motor Speeds!");
    printf("m1: %2.3f, m2: %2.3f, m3: %2.3f, m4: %2.3f \n",mot[0],mot[1],mot[2],mot[3]);
}



BOOST_AUTO_TEST_SUITE_END()


void add_throttles_to_mixing_matrix(double* u, double* mot)
{
    double min, max;

    // 1) Z
    rc_saturate_double(&setpoint.Z_throttle, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
    u[VEC_Z] = setpoint.Z_throttle;
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

void add_throttles_to_mixing_matrix_v2(double* u, double* mot)
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
    // printf("Roll min/max: %2.3f, %2.3f, mot: %2.3f, %2.3f, %2.3f, %2.3f \n",min,max,mot[0],mot[1],mot[2],mot[3]);

    // 3) Pitch (Y)
    mix_check_saturation(VEC_PITCH, mot, &min, &max);
    if (max > MAX_PITCH_COMPONENT) max = MAX_PITCH_COMPONENT;
    if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
    u[VEC_PITCH] = setpoint.pitch_throttle;
    rc_saturate_double(&u[VEC_PITCH], min, max);
    mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
    // printf("Pitch min/max: %2.3f, %2.3f, mot: %2.3f, %2.3f, %2.3f, %2.3f \n",min,max,mot[0],mot[1],mot[2],mot[3]);

    // Z (2nd pass)
    mix_check_saturation(VEC_Z, mot, &min, &max);
    // printf("Z min/max: %2.3f, %2.3f, mot: %2.3f, %2.3f, %2.3f, %2.3f \n",min,max,mot[0],mot[1],mot[2],mot[3]);
    if (max > 0) max = 0;
    if (min < MIN_THRUST_COMPONENT) min = MIN_THRUST_COMPONENT;
    rc_saturate_double(&extra_throttle, min, max);
    mix_add_input(extra_throttle, VEC_Z, mot);
    u[VEC_Z] = baseline_throttle + extra_throttle;
    // printf("Z min/max: %2.3f, %2.3f, mot: %2.3f, %2.3f, %2.3f, %2.3f \n",min,max,mot[0],mot[1],mot[2],mot[3]);

    // 4) Yaw (Z)
    mix_check_saturation(VEC_YAW, mot, &min, &max);
    if (max > MAX_YAW_COMPONENT) max = MAX_YAW_COMPONENT;
    if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
    u[VEC_YAW] = setpoint.yaw_throttle;
    rc_saturate_double(&u[VEC_YAW], min, max);
    mix_add_input(u[VEC_YAW], VEC_YAW, mot);
    // printf("Yaw min/max: %2.3f, %2.3f, mot: %2.3f, %2.3f, %2.3f, %2.3f \n",min,max,mot[0],mot[1],mot[2],mot[3]);

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