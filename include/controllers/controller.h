#ifndef __CONTROLLER__
#define __CONTROLLER__

#include <input_manager.h>
#include <setpoint_manager.h>
#include <state_estimator.h>

void feedback_controller(double* u, double* mot);

int controller_init();

int controller_reset();

#endif /* __CONTROLLER__ */