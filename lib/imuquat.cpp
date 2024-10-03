#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include <sys/time.h>
#include "imuquat.h"
int fd;

long last_loop_time;
Vector3f accel, gyro;
float temperature;
float qw, qx, qy, qz;

void mpu6050_init() {
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x00);
    usleep(100000);
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);
    wiringPiI2CWriteReg8(fd, CONFIG, 0x00);
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 0x18);  // 2000 deg/s
    wiringPiI2CWriteReg8(fd, ACCEL_CONFIG, 0x18); // 16g
    wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);
}

short read_raw_data(int addr) {
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}
// returns true if it's was time to read
void mpu6050_read() {
    accel.x = read_raw_data(ACCEL_XOUT_H) / 2048.0; // 16g
    accel.y = read_raw_data(ACCEL_XOUT_H + 2) / 2048.0;
    accel.z = read_raw_data(ACCEL_XOUT_H + 4) / 2048.0;

    temperature = read_raw_data(ACCEL_XOUT_H + 6) / 340.0 + 36.53;

    gyro.x = read_raw_data(GYRO_XOUT_H) / 16.4; // 2000 deg/s
    gyro.y = read_raw_data(GYRO_XOUT_H + 2) / 16.4;
    gyro.z = read_raw_data(GYRO_XOUT_H + 4) / 16.4;
#ifdef DEBUG
    {
	printf("accel.xyz(%-8.6lf,%-8.6lf,%-8.6lf),\t gyro.xyz(%-8.6lf,%-8.6lf,%-8.6lf) \ttemperature %-2.3lf ->\t",
		    accel.x,	
		    accel.y,	
		    accel.z,	
		    gyro.x,
		    gyro.y,
		    gyro.z,
		    temperature);
    }
#endif
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL);
    long long microseconds = te.tv_sec * 1000000LL + te.tv_usec;
    return microseconds;
}


MadgwickFilter filter = {1.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0};

float invSqrt(float x) {
    union {
        float f;
        std::uint32_t i;
    } conv;

    float halfx = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/* from quake, gives warnings 
 *
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
*/
void madgwick_update(MadgwickFilter* f, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    // Set integration time by time elapsed since last filter update
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long now = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
    f->delta_t = ((now - f->last_update) / 1000000.0f);
    f->last_update = now;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-f->q1 * gx - f->q2 * gy - f->q3 * gz);
    qDot2 = 0.5f * (f->q0 * gx + f->q2 * gz - f->q3 * gy);
    qDot3 = 0.5f * (f->q0 * gy - f->q1 * gz + f->q3 * gx);
    qDot4 = 0.5f * (f->q0 * gz + f->q1 * gy - f->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * f->q0;
        _2q1 = 2.0f * f->q1;
        _2q2 = 2.0f * f->q2;
        _2q3 = 2.0f * f->q3;
        _4q0 = 4.0f * f->q0;
        _4q1 = 4.0f * f->q1;
        _4q2 = 4.0f * f->q2;
        _8q1 = 8.0f * f->q1;
        _8q2 = 8.0f * f->q2;
        q0q0 = f->q0 * f->q0;
        q1q1 = f->q1 * f->q1;
        q2q2 = f->q2 * f->q2;
        q3q3 = f->q3 * f->q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * f->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * f->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * f->q3 - _2q1 * ax + 4.0f * q2q2 * f->q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= f->beta * s0;
        qDot2 -= f->beta * s1;
        qDot3 -= f->beta * s2;
        qDot4 -= f->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    f->q0 += qDot1 * f->delta_t;
    f->q1 += qDot2 * f->delta_t;
    f->q2 += qDot3 * f->delta_t;
    f->q3 += qDot4 * f->delta_t;

    // Normalise quaternion
    recipNorm = invSqrt(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= recipNorm;
    f->q1 *= recipNorm;
    f->q2 *= recipNorm;
    f->q3 *= recipNorm;
#ifdef DEBUG
	printf("quat.0,1,2,3(%-7.4lf,%-7.4lf,%-7.4lf,%-7.4lf) ",
			f->q0,f->q1,f->q2,f->q3);
#endif	
}
// from ai
float quaternion_to_pitch(float qw, float qx, float qy, float qz) {
    float denominator = 1.0f - 2.0f * (qx * qx + qy * qy);
    
    if (denominator < 0.00001f || denominator > 0.99999f) { // add a small value to avoid division by zero
        denominator = 0.00001f; // set the denominator to a small non-zero value
    }
    
    return atan2(2.0f * (qw * qy + qx * qz), denominator);
}

/*float quaternion_to_pitch(float qw, float qx, float qy, float qz) {
    return atan2(2.0f * (qw * qy - qz * qx), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
}*/
bool imuSetup() {
    fd = wiringPiI2CSetup(MPU6050_ADDR);
    if (fd == -1) {
        fprintf(stderr,"Failed to init I2C communication.\n");
        return false;
    }

    mpu6050_init();
    printf("MPU6050 initialized successfully\n");

    struct timeval tv;
    gettimeofday(&tv, NULL);
    last_loop_time = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
    return true;
}
bool GetPitch(float *pitch)   {
    struct timeval tv; 
	gettimeofday(&tv, NULL);
	long long current_time = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
	if (current_time - last_loop_time >= LOOP_INTERVAL) {
		last_loop_time = current_time;
		mpu6050_read();
		madgwick_update(&filter, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);
		*pitch = quaternion_to_pitch(filter.q0, filter.q1, filter.q2, filter.q3);
		return true;	
	}
	else printf("waiting %lld\n",current_time - last_loop_time);	
	return false;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
