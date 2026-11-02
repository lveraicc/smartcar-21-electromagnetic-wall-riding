#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"


typedef struct
{
    float kp;
    float ki;

    float error;
    float last_error;

    float output;

    float max_output;

} IncrementalPID;


typedef struct
{
    float kp;
    float kd;

    float error;
    float last_error;

    float output;
	
	   float d;   

    float max_output;

} PositionPID;




void Incremental_PID_Init(IncrementalPID *pid,float kp,float ki,float max_output);

float Incremental_PID_Calc(IncrementalPID *pid,float target,float feedback);



void Position_PID_Init(PositionPID *pid,float kp,float kd,float max_output);

float Position_PID_Calc(PositionPID *pid, float target,float feedback);

#endif