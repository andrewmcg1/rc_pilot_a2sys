#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>

#include <rc/servo.h>
#include <rc/time.h>

#include <setpoint_manager.h>
#include <delta_arm.h>

#define START_LOC 0
#define SPEED .5


void* __send_pulses(void* servo_);


void servos_init(int* ch, int num_ch)
{
    if(!servos.initialized)
    {
        for(int i = 0; i < num_ch; i++)
        {
            servos.running[ch[i]-1] = 1;
            servos.width_goal[ch[i]-1] = (((US_MIN + US_MAX) / 2) + START_LOC * (US_MAX - US_MIN) / (MAX_ANGLE * 2));
            servos.width[ch[i]-1] = servos.width_goal[ch[i]-1];
        }
        if(!rc_servo_init() && !rc_servo_power_rail_en(1))
        {    
            rc_pthread_create(&(servos.send_pulse_thread), __send_pulses, (void*)&servos, 0, 0);
            rc_usleep(1000000);
        }
        else
        {
            printf("ERROR: Failed to initialize servos\n");
            exit(1);
        }
    }
    else
    {
        printf("ERROR: Servos already initialized\n");
        exit(1);
    }
}

void* __send_pulses(void* servo_)
{
    int pulse_success = 0;
    servo_t* servos_ = (servo_t*)servo_;
    while(1)
    {    
        for(int i = 0; i < 8; i++)
        {
            pulse_success = rc_servo_send_pulse_us(i+1, (int)servos_->width[i]);
            if(pulse_success)
            {
                printf("ERROR: Failed to send pulse\n");
            }
            else if(servos_->running[i] == 1 && (int)servos_->width[i] != servos_->width_goal[i])
            {
                servos_->width[i] += (servos_->width[i] < servos_->width_goal[i]) ? SPEED : -1 * SPEED;
            }   
        }
        rc_usleep(2000);   // 500 Hz
    }
}

void servo_set_angle(int ch, double angle)
{
    if (fabs(angle) > MAX_ANGLE)
    {
        printf("ERROR: Invalid angle: %lf\n", angle);        
        servos.running[ch-1] = 0;
        rc_pthread_timed_join(servos.send_pulse_thread, NULL, 1.0); // wait for it to stop
        rc_servo_cleanup();
        exit(1);
    }
    servos.width_goal[ch-1] = (((US_MIN + US_MAX) / 2) + angle * (US_MAX - US_MIN) / (MAX_ANGLE * 2));
    rc_usleep(10000);
}


// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(point_t* location) 
{
    double theta1 = location->theta1;
    double theta2 = location->theta2;
    double theta3 = location->theta3;

    double t = (f-e)*tan30/2;
    double dtr = pi/(double)180.0;
    
    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;
    
    double y1 = -(t + rf*cos(theta1));
    double z1 = -rf*sin(theta1);
    
    double y2 = (t + rf*cos(theta2))*sin30;
    double x2 = y2*tan60;
    double z2 = -rf*sin(theta2);
    
    double y3 = (t + rf*cos(theta3))*sin30;
    double x3 = -y3*tan60;
    double z3 = -rf*sin(theta3);
    
    double dnm = (y2-y1)*x3-(y3-y1)*x2;
    
    double w1 = y1*y1 + z1*z1;
    double w2 = x2*x2 + y2*y2 + z2*z2;
    double w3 = x3*x3 + y3*y3 + z3*z3;
        
    // x = (a1*z + b1)/dnm
    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
    
    // y = (a2*z + b2)/dnm;
    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
    double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
    
    // a*z^2 + b*z + c = 0
    double a = a1*a1 + a2*a2 + dnm*dnm;
    double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
    
    // discriminant
    double d = b*b - (double)4.0*a*c;
    if (d < 0) return -1; // non-existing point
    
    location->z = -(double)0.5*(b+sqrt(d))/a;
    location->x = (a1*(location->z) + b1)/dnm;
    location->y = (a2*(location->z) + b2)/dnm;
    return 0;
}
    
    // inverse kinematics
    // helper functions, calculates angle theta1 (for YZ-plane)
int delta_calcAngleYZ(double x0, double y0, double z0, double *theta)
{
    double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735    * e;    // shift center to edge

    // z = a + b*y
    double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
    double b = (y1-y0)/z0;

    // discriminant
    double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
    if (d < 0) 
        return -1; // non-existing point

    double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
    double zj = a + b*yj;
    *theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
    return 0;
}
 
// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(point_t* location)
{
    location->theta1 = location->theta2 = location->theta3 = 0;
    int status = delta_calcAngleYZ(location->x, location->y, location->z, &location->theta1);
    if (status == 0) 
        status = delta_calcAngleYZ(location->x*cos120 + location->y*sin120, location->y*cos120-location->x*sin120, location->z, &location->theta2);  // rotate coords to +120 deg
    if (status == 0) 
        status = delta_calcAngleYZ(location->x*cos120 - location->y*sin120, location->y*cos120+location->x*sin120, location->z, &location->theta3);  // rotate coords to -120 deg
    return status;
}

int delta_validAngles(point_t* location)
{
    if(fabs(location->theta1-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta2-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta3-MAX_ANGLE) < MAX_ANGLE)
        return 0;
    else
        return -1;
}

void delta_update_angles(point_t* location)
{
    if(fabs(location->theta1-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta2-MAX_ANGLE) < MAX_ANGLE && fabs(location->theta3-MAX_ANGLE) < MAX_ANGLE)
    {
        servo_set_angle(1, location->theta1-MAX_ANGLE);
        servo_set_angle(2, location->theta2-MAX_ANGLE);
        servo_set_angle(3, location->theta3-MAX_ANGLE);
    }
    else
        printf("Invalid Angles: %f %f %f\n", location->theta1-MAX_ANGLE, location->theta2-MAX_ANGLE, location->theta3-MAX_ANGLE);
}

int __make_loiter_wp_file(char* filename)
{
    FILE *fp;
    fp = fopen(filename, "w");
    if(fp == NULL)
    {
        printf("Error opening file!\n");
        return -1;
    }
    fprintf(fp, "\n");
    return 0;
}