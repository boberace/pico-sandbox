#ifndef PID_H
#define PID_H

// Structure to hold PID controller data
typedef struct {
    float setpoint;             // Desired value to maintain
    float kp;                   // Proportional constant
    float ki;                   // Integral constant
    float kd;                   // Derivative constant
    float difference_error;     // Error between setpoint and sensor input
    float accumulative_error;   // Accumulated error for integral term
    float rate_error;           // Rate of change of error
    float controller_output;    // Output of the PID controller
} PID;

// Function prototypes
void pid_init(PID *pid, float kp, float ki, float kd);
float pid_output_update(PID *pid, float sensor_input, float time_change);
void pid_set_setpoint(PID *pid, float setpoint);
float pid_get_setpoint(PID *pid);
void pid_set_kp(PID *pid, float kp);
void pid_set_ki(PID *pid, float ki);
void pid_set_kd(PID *pid, float kd);
float pid_get_kp(PID *pid);
float pid_get_ki(PID *pid);
float pid_get_kd(PID *pid);

#endif // PID_H