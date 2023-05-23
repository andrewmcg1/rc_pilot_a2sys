/**
 *  initializes PID term filtering 
 *  starts setpoint manager, to guide quadcopter to waypoint positions
 *  handles mode definitions, which define how the drone is controller, whether by the controller or autonomously
 *  handles PID control loops for altitude control, horizontal control, rpy, and rpy rate.
 *  adds throttles to mixing matrixes
 */



#ifndef __CONTROLLER__
#define __CONTROLLER__

#include <input_manager.h>
#include <setpoint_manager.h>
#include <state_estimator.h>

void feedback_controller(double* u, double* mot);

int controller_init();

int controller_reset();

#endif /* __CONTROLLER__ */