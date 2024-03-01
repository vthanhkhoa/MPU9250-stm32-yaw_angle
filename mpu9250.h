#ifndef DRIVERS_MPU_9250_H_
#define DRIVERS_MPU_9250_H_

/*********************
 *      INCLUDES
 *********************/

#include <stdint.h>
#include "main.h"

/*********************
 *      DEFINES
 *********************/

#define MPU9250_ADDR 0xD0


#define MPU9250_SMPRT_DIV 0X19
#define MPU9250_WHO_AM_I 0X75
#define MPU9250_CONFIG 0X1A
#define MPU9250_GYRO_CONFIG 0X1B
#define MPU9250_ACCEL_CONFIG 0X1C
#define MPU9250_INT_PIN_CFG 0X37
#define MPU9250_INT_ENABLE 0X38
#define MPU9250_INT_STATUS 0X3A
#define MPU9250_ACCEL_XOUT_H 0X3B
#define MPU9250_ACCEL_XOUT_L 0X3C
#define MPU9250_PWR_MGMT_1 0X6B //most important
//#define MPU9250_INT_PIN_CFG             0x37        /*!< Interrupt pin/bypass enable configuration */
//#define MPU9250_ADDR                    (0x68<<1)   /*!< MPU9250 Address */

/**********************
 *      TYPEDEFS
 **********************/
// float LSB_Sensitivity_ACC;
// float LSB_Sensitivity_GYRO;
// uint16_t error;
extern I2C_HandleTypeDef hi2c1;

typedef struct _MPU9250
{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;

	short temperature_raw;

	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	short mag_x_raw;
	short mag_y_raw;
	short mag_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;

	float temperature;

	float gyro_x;
	float gyro_y;
	float gyro_z;

    float Filt_accx;
	float Filt_accy;
	float Filt_accz;

    float Filt_gyx;
	float Filt_gyy;
	float Filt_gyz;

    float cal_gyx;
	float cal_gyy;
	float cal_gyz;
} MPU9250_t;

typedef struct imu_9250 imu_9250_t;

struct imu_9250
{
	//	I2C_HandleTypeDef *hi2c;
	void (*get_data)(imu_9250_t *const imu_p);
	MPU9250_t pt1_p;
};

/**********************
 *     OPERATION
 **********************/

imu_9250_t *IMU_9250_Create();
void IMU_9250_Destroy(imu_9250_t *const imu_p);

#endif /* DRIVERS_MPU_9250_H_ */
