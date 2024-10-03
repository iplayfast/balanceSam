#ifndef IMUQUAT_H
#define IMUQUAT_H

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

#define RAD_TO_DEG 57.29578
#define LOOP_INTERVAL 10000 // 10ms in microseconds


typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

void mpu6050_init();

short read_raw_data(int addr);

void mpu6050_read();

bool GetPitch(float *pitch);
long long current_timestamp();
// Madgwick filter implementation
typedef struct {
    float q0, q1, q2, q3;  // Quaternion
    float beta;            // Algorithm gain
    float delta_t;
    long long last_update;
} MadgwickFilter;

float invSqrt(float x);
void madgwick_update(MadgwickFilter* f, float gx, float gy, float gz, float ax, float ay, float az);

float quaternion_to_pitch(float qw, float qx, float qy, float qz);
bool imuSetup();
void imuLoop();
long map(long x, long in_min, long in_max, long out_min, long out_max) ;
#endif
