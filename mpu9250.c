/*********************
 *      INCLUDES
 *********************/

#include "mpu9250.h"
#include <stdlib.h>

float LSB_Sensitivity_ACC;
float LSB_Sensitivity_GYRO;
extern uint16_t error;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void MPU9250_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU9250_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU9250_Readbyte(uint8_t reg_addr, uint8_t *data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU9250_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}
/// @brief
/// @param
void start_imu(void)
{
	HAL_Delay(50);
	uint8_t who_am_i = 0;
	MPU9250_Readbyte(MPU9250_WHO_AM_I, &who_am_i);
	if (who_am_i == 0x71)
	{
		error = 20;
	}
	else
	{
		error = 404;
		while (1);
	}

	// Reset the whole module before initialization
	MPU9250_Writebyte(MPU9250_PWR_MGMT_1, 0x1 << 7);
	HAL_Delay(100);

	// Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU9250*/
	MPU9250_Writebyte(MPU9250_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	// Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	//	MPU9250_Writebyte(MPU9250_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz
	MPU9250_Writebyte(MPU9250_SMPRT_DIV, 39); // Sample Rate = 200Hz		//**********************
	HAL_Delay(50);

	// FSYNC and DLPF setting
	/*DLPF is set to 0*/
	MPU9250_Writebyte(MPU9250_CONFIG, 0x00); //**********************
	HAL_Delay(50);

	// GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x00;
	MPU9250_Writebyte(MPU9250_GYRO_CONFIG, FS_SCALE_GYRO << 3);
	HAL_Delay(50);

	// ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x00;
	MPU9250_Writebyte(MPU9250_ACCEL_CONFIG, FS_SCALE_ACC << 3);
	HAL_Delay(50);

	MPU9250_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
}

void MPU9250_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch (FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch (FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

void MPU9250_ProcessData(imu_9250_t *mpu9250)
{
	MPU9250_Get6AxisRawData(mpu9250);
	MPU9250_DataConvert(mpu9250);
}

void MPU9250_Get6AxisRawData(imu_9250_t *mpu9250)
{
	uint8_t data[14];
	MPU9250_Readbytes(MPU9250_ACCEL_XOUT_H, 14, data);

	mpu9250->pt1_p.acc_x_raw = (data[0] << 8) | data[1];
	mpu9250->pt1_p.acc_y_raw = (data[2] << 8) | data[3];
	mpu9250->pt1_p.acc_z_raw = (data[4] << 8) | data[5];

	mpu9250->pt1_p.temperature_raw = (data[6] << 8) | data[7];

	mpu9250->pt1_p.gyro_x_raw = ((data[8] << 8) | data[9]);
	mpu9250->pt1_p.gyro_y_raw = ((data[10] << 8) | data[11]);
	mpu9250->pt1_p.gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU9250_DataConvert(imu_9250_t *mpu9250)
{
	// printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu9250->pt1_p.acc_x = mpu9250->pt1_p.acc_x_raw / LSB_Sensitivity_ACC;
	mpu9250->pt1_p.acc_y = mpu9250->pt1_p.acc_y_raw / LSB_Sensitivity_ACC;
	mpu9250->pt1_p.acc_z = mpu9250->pt1_p.acc_z_raw / LSB_Sensitivity_ACC;

	mpu9250->pt1_p.temperature = (float)(mpu9250->pt1_p.temperature_raw) / 340 + 36.53;

	mpu9250->pt1_p.gyro_x = (mpu9250->pt1_p.gyro_x_raw) / LSB_Sensitivity_GYRO;
	mpu9250->pt1_p.gyro_y = (mpu9250->pt1_p.gyro_y_raw) / LSB_Sensitivity_GYRO;
	mpu9250->pt1_p.gyro_z = (mpu9250->pt1_p.gyro_z_raw - mpu9250->pt1_p.cal_gyz) / (LSB_Sensitivity_GYRO * 10);
}

void calibrateGyro(imu_9250_t *mpu9250, uint16_t numCalPoints)
{
	// Init
	int32_t xx = 0;
	int32_t yy = 0;
	int32_t zz = 0;

	// Zero guard
	if (numCalPoints == 0)
	{
		numCalPoints = 1;
	}

	// Save specified number of points
	for (uint16_t ii = 0; ii < numCalPoints; ii++)
	{
		MPU9250_Get6AxisRawData(mpu9250);
		xx += mpu9250->pt1_p.gyro_x_raw;
		yy += mpu9250->pt1_p.gyro_y_raw;
		zz += mpu9250->pt1_p.gyro_z_raw;
		HAL_Delay(3);
	}

	// Average the saved data points to find the gyroscope offset
	mpu9250->pt1_p.cal_gyx = (float)xx / (float)numCalPoints;
	mpu9250->pt1_p.cal_gyy = (float)yy / (float)numCalPoints;
	mpu9250->pt1_p.cal_gyz = (float)zz / (float)numCalPoints;
}

void IMU_9250_Init(imu_9250_t *imu_p, void (*get_data_func)(imu_9250_t *imu_p))
{
	imu_p->get_data = get_data_func;
	start_imu();
}

/**
 * The function `MPU_calibrateGyro` calibrates the gyroscope of an MPU9250 sensor by averaging a
 * specified number of raw data points.
 *
 * @param mpu9250 The `mpu9250` parameter is a pointer to a structure of type `MPU9250_t`, which likely
 * contains data related to an MPU9250 sensor module. The function `MPU_calibrateGyro` calibrates the
 * gyroscope of the MPU9250 sensor by taking
 * @param numCalPoints The `numCalPoints` parameter in the `MPU_calibrateGyro` function represents the
 * number of data points to be collected for calibrating the gyroscope. It specifies how many readings
 * will be taken to calculate the average gyroscope offset values.
 */

/**
 * The function creates and initializes a new instance of the IMU_9250 structure.
 *
 * @return a pointer to a structure of type imu_9250_t.
 */
imu_9250_t *IMU_9250_Create()
{
	imu_9250_t *imu_p = malloc(sizeof(imu_9250_t));
	if (imu_p != NULL)
	{
		IMU_9250_Init(imu_p, MPU9250_ProcessData);
	}
	return imu_p;
}

/**
 * The function IMU_9250_Destroy frees the memory allocated for an imu_9250_t structure.
 *
 * @param imu_p imu_p is a pointer to a structure of type imu_9250_t.
 */
void IMU_9250_Destroy(imu_9250_t *const imu_p)
{
	free(imu_p);
}
