#include <math.h>
#include "pid.h"

// Initialize the PID controller
void pid_init(PID *pid, float kp, float ki, float kd) {
    pid->setpoint = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->difference_error = 0;
    pid->accumulative_error = 0;
    pid->rate_error = 0;
    pid->controller_output = 0;
}

// Update the PID controller output
float pid_output_update(PID *pid, float sensor_input, float time_change) {
    float previous_difference_error = pid->difference_error;
    float previous_controller_output = pid->controller_output;

    pid->difference_error = pid->setpoint - sensor_input;

    // Prevent integral windup
    if (previous_controller_output > -1 && previous_controller_output < 1) {
        pid->accumulative_error += pid->difference_error * time_change;
    }

    pid->rate_error = (pid->difference_error - previous_difference_error) / time_change;

    pid->controller_output = pid->kp * pid->difference_error +
                             pid->ki * pid->accumulative_error +
                             pid->kd * pid->rate_error;

    // Clamp output to range [-1, 1]
    if (pid->controller_output > 1) {
        return 1;
    } else if (pid->controller_output < -1) {
        return -1;
    } else {
        return pid->controller_output;
    }
}

// Set the setpoint for the PID controller
void pid_set_setpoint(PID *pid, float setpoint) {
    pid->setpoint = setpoint;
}

// Get the setpoint of the PID controller
float pid_get_setpoint(PID *pid) {
    return pid->setpoint;
}

// Set the proportional constant
void pid_set_kp(PID *pid, float kp) {
    pid->kp = kp;
}

// Set the integral constant
void pid_set_ki(PID *pid, float ki) {
    pid->ki = ki;
}

// Set the derivative constant
void pid_set_kd(PID *pid, float kd) {
    pid->kd = kd;
}

// Get the proportional constant
float pid_get_kp(PID *pid) {
    return pid->kp;
}

// Get the integral constant
float pid_get_ki(PID *pid) {
    return pid->ki;
}

// Get the derivative constant
float pid_get_kd(PID *pid) {
    return pid->kd;
}