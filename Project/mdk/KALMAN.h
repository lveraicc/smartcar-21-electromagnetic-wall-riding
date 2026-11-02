#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "zf_common_headfile.h"

typedef struct
{
    int16 acc_raw[3];
    int16 gyro_raw[3];
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float pitch_raw;
    float roll_raw;
    float pitch_kalman;
    float roll_kalman;
    float pitch_360;
    float roll_360;
    uint8 calibrated;
} KalmanAngleData;

void KalmanGyro_Init(void);
void KalmanGyro_SetDt(float dt_seconds);
void KalmanGyro_SetNoise(float q_angle, float q_bias, float r_measure);
void KalmanGyro_Update(const int16 acc_raw[3], const int16 gyro_raw[3], KalmanAngleData *out);
void KalmanGyro_UpdateFromIMU660RB(KalmanAngleData *out);
const KalmanAngleData *KalmanGyro_GetData(void);
int32 KalmanGyro_GetPitchRawValue(void);
int32 KalmanGyro_GetPitchValue(void);
int32 KalmanGyro_GetPitch360Value(void);
void KalmanGyro_ResetYaw(void);
int32 KalmanGyro_GetYawValue(void);

#endif
