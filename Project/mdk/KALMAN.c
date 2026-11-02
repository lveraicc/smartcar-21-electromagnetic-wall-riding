#include "KALMAN.h"

#include "zf_device_imu660rb.h"
#include <math.h>

#define RAD_TO_DEG                 (57.2957795f)
#define DEG_TO_OUTPUT_SCALE        (100.0f)
#define DEFAULT_DT_SECONDS         (0.01f)
#define GYRO_BIAS_SAMPLE_COUNT     (50U)
#define KALMAN_Q_ANGLE             (0.01f)
#define KALMAN_Q_BIAS              (0.003f)
#define KALMAN_R_MEASURE           (0.01f)
#define KALMAN_PITCH_360_OFFSET    (180.0f)
#define KALMAN_PITCH_GYRO_SIGN     (-1.0f)
#define KALMAN_ROLL_GYRO_SIGN      (1.0f)

typedef struct
{
    float q_angle;
    float q_bias;
    float r_measure;
    float angle;
    float bias;
    float p[2][2];
    uint8 initialized;
} KalmanAngleFilter;

static KalmanAngleFilter s_pitch_filter;
static KalmanAngleFilter s_roll_filter;
static KalmanAngleData s_angle_data;
static float s_dt_seconds = DEFAULT_DT_SECONDS;
static float s_yaw_relative = 0.0f;
static float s_gyro_bias[3];
static float s_gyro_bias_sum[3];
static uint16 s_gyro_bias_count;
static uint8 s_gyro_bias_ready;

static float KalmanAbs(float value)
{
    if(value < 0.0f)
    {
        return -value;
    }
    return value;
}

static float KalmanNormalize360(float angle_deg)
{
    while(angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    while(angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static int32 KalmanDegToOutputValue(float angle_deg)
{
    float value;

    value = angle_deg * DEG_TO_OUTPUT_SCALE;
    if(value >= 0.0f)
    {
        return (int32)(value + 0.5f);
    }
    return (int32)(value - 0.5f);
}

static int32 KalmanDegTo360OutputValue(float angle_deg)
{
    int32 value;

    value = KalmanDegToOutputValue(KalmanNormalize360(angle_deg));
    if(value >= 36000)
    {
        value -= 36000;
    }
    return value;
}

static void KalmanAngle_Reset(KalmanAngleFilter *filter)
{
    filter->q_angle = KALMAN_Q_ANGLE;
    filter->q_bias = KALMAN_Q_BIAS;
    filter->r_measure = KALMAN_R_MEASURE;
    filter->angle = 0.0f;
    filter->bias = 0.0f;
    filter->p[0][0] = 0.0f;
    filter->p[0][1] = 0.0f;
    filter->p[1][0] = 0.0f;
    filter->p[1][1] = 0.0f;
    filter->initialized = 0;
}

static void KalmanAngle_Start(KalmanAngleFilter *filter, float angle)
{
    filter->angle = angle;
    filter->bias = 0.0f;
    filter->p[0][0] = 0.0f;
    filter->p[0][1] = 0.0f;
    filter->p[1][0] = 0.0f;
    filter->p[1][1] = 0.0f;
    filter->initialized = 1;
}

static float KalmanAngle_Update(KalmanAngleFilter *filter, float acc_angle, float gyro_rate)
{
    float rate;
    float s;
    float k0;
    float k1;
    float y;
    float p00_temp;
    float p01_temp;

    if(filter->initialized == 0)
    {
        filter->angle = acc_angle;
        filter->initialized = 1;
        return filter->angle;
    }

    rate = gyro_rate - filter->bias;
    filter->angle += s_dt_seconds * rate;

    filter->p[0][0] += s_dt_seconds *
        (s_dt_seconds * filter->p[1][1] - filter->p[0][1] - filter->p[1][0] + filter->q_angle);
    filter->p[0][1] -= s_dt_seconds * filter->p[1][1];
    filter->p[1][0] -= s_dt_seconds * filter->p[1][1];
    filter->p[1][1] += filter->q_bias * s_dt_seconds;

    s = filter->p[0][0] + filter->r_measure;
    k0 = filter->p[0][0] / s;
    k1 = filter->p[1][0] / s;

    y = acc_angle - filter->angle;
    filter->angle += k0 * y;
    filter->bias += k1 * y;

    p00_temp = filter->p[0][0];
    p01_temp = filter->p[0][1];

    filter->p[0][0] -= k0 * p00_temp;
    filter->p[0][1] -= k0 * p01_temp;
    filter->p[1][0] -= k1 * p00_temp;
    filter->p[1][1] -= k1 * p01_temp;

    return filter->angle;
}

void KalmanGyro_Init(void)
{
    KalmanAngle_Reset(&s_pitch_filter);
    KalmanAngle_Reset(&s_roll_filter);

    s_angle_data.acc_raw[0] = 0;
    s_angle_data.acc_raw[1] = 0;
    s_angle_data.acc_raw[2] = 0;
    s_angle_data.gyro_raw[0] = 0;
    s_angle_data.gyro_raw[1] = 0;
    s_angle_data.gyro_raw[2] = 0;
    s_angle_data.acc_x = 0.0f;
    s_angle_data.acc_y = 0.0f;
    s_angle_data.acc_z = 0.0f;
    s_angle_data.gyro_x = 0.0f;
    s_angle_data.gyro_y = 0.0f;
    s_angle_data.gyro_z = 0.0f;
    s_angle_data.gyro_bias_x = 0.0f;
    s_angle_data.gyro_bias_y = 0.0f;
    s_angle_data.gyro_bias_z = 0.0f;
    s_angle_data.pitch_raw = 0.0f;
    s_angle_data.roll_raw = 0.0f;
    s_angle_data.pitch_kalman = 0.0f;
    s_angle_data.roll_kalman = 0.0f;
    s_angle_data.pitch_360 = 0.0f;
    s_angle_data.roll_360 = 0.0f;
    s_angle_data.calibrated = 0;

    s_gyro_bias[0] = 0.0f;
    s_gyro_bias[1] = 0.0f;
    s_gyro_bias[2] = 0.0f;
    s_gyro_bias_sum[0] = 0.0f;
    s_gyro_bias_sum[1] = 0.0f;
    s_gyro_bias_sum[2] = 0.0f;
    s_gyro_bias_count = 0;
    s_gyro_bias_ready = 0;
    s_dt_seconds = DEFAULT_DT_SECONDS;
    s_yaw_relative = 0.0f;
}

void KalmanGyro_SetDt(float dt_seconds)
{
    if(dt_seconds > 0.0f)
    {
        s_dt_seconds = dt_seconds;
    }
}

void KalmanGyro_SetNoise(float q_angle, float q_bias, float r_measure)
{
    if(q_angle > 0.0f)
    {
        s_pitch_filter.q_angle = q_angle;
        s_roll_filter.q_angle = q_angle;
    }

    if(q_bias > 0.0f)
    {
        s_pitch_filter.q_bias = q_bias;
        s_roll_filter.q_bias = q_bias;
    }

    if(r_measure > 0.0f)
    {
        s_pitch_filter.r_measure = r_measure;
        s_roll_filter.r_measure = r_measure;
    }
}

void KalmanGyro_Update(const int16 acc_raw[3], const int16 gyro_raw[3], KalmanAngleData *out)
{
    float acc_z_abs;
    float gyro_dps[3];

    s_angle_data.acc_raw[0] = acc_raw[0];
    s_angle_data.acc_raw[1] = acc_raw[1];
    s_angle_data.acc_raw[2] = acc_raw[2];
    s_angle_data.gyro_raw[0] = gyro_raw[0];
    s_angle_data.gyro_raw[1] = gyro_raw[1];
    s_angle_data.gyro_raw[2] = gyro_raw[2];

    s_angle_data.acc_x = imu660rb_acc_transition(acc_raw[0]);
    s_angle_data.acc_y = imu660rb_acc_transition(acc_raw[1]);
    s_angle_data.acc_z = imu660rb_acc_transition(acc_raw[2]);
    gyro_dps[0] = imu660rb_gyro_transition(gyro_raw[0]);
    gyro_dps[1] = imu660rb_gyro_transition(gyro_raw[1]);
    gyro_dps[2] = imu660rb_gyro_transition(gyro_raw[2]);

    acc_z_abs = KalmanAbs(s_angle_data.acc_z);
    if(acc_z_abs < 0.000001f)
    {
        acc_z_abs = 0.000001f;
    }

    s_angle_data.pitch_raw = (float)(atan2((double)(-s_angle_data.acc_y), (double)acc_z_abs) * RAD_TO_DEG);
    s_angle_data.roll_raw = (float)(atan2((double)(s_angle_data.acc_x), (double)acc_z_abs) * RAD_TO_DEG);
    s_angle_data.pitch_360 = KalmanNormalize360((float)(atan2((double)(-s_angle_data.acc_y), (double)s_angle_data.acc_z) * RAD_TO_DEG));
    s_angle_data.roll_360 = KalmanNormalize360((float)(atan2((double)(s_angle_data.acc_x), (double)s_angle_data.acc_z) * RAD_TO_DEG));

    if(s_gyro_bias_ready == 0)
    {
        s_gyro_bias_sum[0] += gyro_dps[0];
        s_gyro_bias_sum[1] += gyro_dps[1];
        s_gyro_bias_sum[2] += gyro_dps[2];
        s_gyro_bias_count++;

        s_gyro_bias[0] = s_gyro_bias_sum[0] / (float)s_gyro_bias_count;
        s_gyro_bias[1] = s_gyro_bias_sum[1] / (float)s_gyro_bias_count;
        s_gyro_bias[2] = s_gyro_bias_sum[2] / (float)s_gyro_bias_count;

        if(s_gyro_bias_count >= GYRO_BIAS_SAMPLE_COUNT)
        {
            s_gyro_bias_ready = 1;
            KalmanAngle_Start(&s_pitch_filter, s_angle_data.pitch_raw);
            KalmanAngle_Start(&s_roll_filter, s_angle_data.roll_raw);
        }
    }

    s_angle_data.gyro_bias_x = s_gyro_bias[0];
    s_angle_data.gyro_bias_y = s_gyro_bias[1];
    s_angle_data.gyro_bias_z = s_gyro_bias[2];
    s_angle_data.calibrated = s_gyro_bias_ready;
    s_angle_data.gyro_x = gyro_dps[0] - s_gyro_bias[0];
    s_angle_data.gyro_y = gyro_dps[1] - s_gyro_bias[1];
    s_angle_data.gyro_z = gyro_dps[2] - s_gyro_bias[2];

	if(s_gyro_bias_ready != 0)
	{
		s_yaw_relative += s_angle_data.gyro_z * s_dt_seconds;
	}

    if(s_gyro_bias_ready == 0)
    {
        s_angle_data.pitch_kalman = s_angle_data.pitch_raw;
        s_angle_data.roll_kalman = s_angle_data.roll_raw;
    }
    else
    {
        s_angle_data.pitch_kalman = KalmanAngle_Update(&s_pitch_filter,
            s_angle_data.pitch_raw,
            KALMAN_PITCH_GYRO_SIGN * s_angle_data.gyro_x);
        s_angle_data.roll_kalman = KalmanAngle_Update(&s_roll_filter,
            s_angle_data.roll_raw,
            KALMAN_ROLL_GYRO_SIGN * s_angle_data.gyro_y);
    }

    if(out != 0)
    {
        *out = s_angle_data;
    }
}

void KalmanGyro_UpdateFromIMU660RB(KalmanAngleData *out)
{
    int16 acc_raw[3];
    int16 gyro_raw[3];

    imu660rb_get_acc();
    imu660rb_get_gyro();

    acc_raw[0] = imu660rb_acc_x;
    acc_raw[1] = imu660rb_acc_y;
    acc_raw[2] = imu660rb_acc_z;
    gyro_raw[0] = imu660rb_gyro_x;
    gyro_raw[1] = imu660rb_gyro_y;
    gyro_raw[2] = imu660rb_gyro_z;

    KalmanGyro_Update(acc_raw, gyro_raw, out);
}

const KalmanAngleData *KalmanGyro_GetData(void)
{
    return &s_angle_data;
}

int32 KalmanGyro_GetPitchRawValue(void)
{
    return KalmanDegToOutputValue(s_angle_data.pitch_raw);
}

int32 KalmanGyro_GetPitchValue(void)
{
    return KalmanDegToOutputValue(s_angle_data.pitch_kalman);
}

int32 KalmanGyro_GetPitch360Value(void)
{
    return KalmanDegTo360OutputValue(KALMAN_PITCH_360_OFFSET - s_angle_data.pitch_360);
}

void KalmanGyro_ResetYaw(void)
{
	s_yaw_relative = 0.0f;
}

int32 KalmanGyro_GetYawValue(void)
{
	return KalmanDegToOutputValue(s_yaw_relative);
}
