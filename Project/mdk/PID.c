#include "pid.h"

static float limit(float value, float max)
{
    if(value > max)
        value = max;

    if(value < -max)
        value = -max;

    return value;
}


void Incremental_PID_Init(IncrementalPID *pid,float kp,float ki,float max_output)
{
    pid->kp = kp;
    pid->ki = ki;

    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->output = 0.0f;

    pid->max_output = max_output;
}

float Incremental_PID_Calc(IncrementalPID *pid,float target,float feedback)
{
    float increment;

    pid->error = target - feedback;

    increment =
            pid->kp * (pid->error - pid->last_error)
          + pid->ki * pid->error;

    pid->output += increment;

    pid->output = limit(pid->output,
                        pid->max_output);

    pid->last_error = pid->error;

    return pid->output;
}


void Position_PID_Init(PositionPID *pid,float kp,float kd,float max_output)
{
    pid->kp = kp;
    pid->kd = kd;

    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->d = 0.0f;         

    pid->output = 0.0f;

    pid->max_output = max_output;
}

float Position_PID_Calc(PositionPID *pid,float target,float feedback)
{
    float derivative;

    pid->error = target - feedback;

    derivative = pid->error - pid->last_error;

    pid->d = pid->d * 0.7f
           + derivative * 0.3f;

    pid->output =
            pid->kp * pid->error
          + pid->kd * pid->d;

 
    pid->output = limit(pid->output,
                        pid->max_output);

    pid->last_error = pid->error;

    return pid->output;
}