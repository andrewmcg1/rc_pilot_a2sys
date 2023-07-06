#ifndef DELTA_ARM_H
#define DELTA_ARM_H

#include <rc/pthread.h>


#define MAX_ANGLE 68

#define US_MAX 2100
#define US_MIN 900


typedef struct {
    int initialized;
    pthread_t send_pulse_thread;

    int running[8];
    
    int width_goal[8];
    float width[8];
} servo_t;


typedef struct {
    double x;
    double y;
    double z;

    double theta1;
    double theta2;
    double theta3;
} point_t;

servo_t servos;
extern point_t delta_end_effector;


void servo_set_angle(int ch, double angle);
void servos_init(int* ch, int num_ch);


// Delta arm function declarations
int delta_calcForward(point_t* location);
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta);
int delta_calcInverse(point_t* location);
void delta_update_angles(point_t* location);

// Robot geometry (assumes identical links) 
// --> watch these globals - may want to add identifiers (e.g., "delta_f", "delta_e"...)
const double f = 130;     // base reference triangle side length
const double e = 43;     // end effector reference triangle side length
const double rf = 311.15;    // Top link length
const double re = 381;    // Bottom link length

// trigonometric constants
const double pi = 3.141592653;    // PI
const double sqrt3 = 1.732051;    // sqrt(3)
const double sin120 = 0.866025;   // sqrt(3)/2
const double cos120 = -0.5;        
const double tan60 = 1.732051;    // sqrt(3)
const double sin30 = 0.5;
const double tan30 = 0.57735;     // 1/sqrt(3)

#endif