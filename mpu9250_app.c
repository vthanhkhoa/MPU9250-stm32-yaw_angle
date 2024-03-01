#include "mpu9250_app.h"
#include "math.h"

static float ddt = 1 / 200.f; // Sample rate is 200Hz

void CalculateAccAngle(Struct_Angle *Angle, imu_9250_t *mpu9250)
{
	Angle->acc_roll = atan(-mpu9250->pt1_p.acc_x / sqrt(pow(mpu9250->pt1_p.acc_y, 2) + pow(mpu9250->pt1_p.acc_z, 2))) * RADIAN_TO_DEGREE;
	Angle->acc_pitch = atan(mpu9250->pt1_p.acc_y / sqrt(pow(mpu9250->pt1_p.acc_x, 2) + pow(mpu9250->pt1_p.acc_z, 2))) * RADIAN_TO_DEGREE;
}

void CalculateGyroAngle(Struct_Angle *Angle, imu_9250_t *mpu9250)
{
	Angle->gyro_roll += mpu9250->pt1_p.gyro_x * ddt;
	Angle->gyro_pitch += mpu9250->pt1_p.gyro_y * ddt;
	Angle->gyro_yaw += mpu9250->pt1_p.gyro_z * ddt;
}