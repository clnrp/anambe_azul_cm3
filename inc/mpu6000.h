/*
 * mpu6050.h
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "spi.h"

#define	SMPLRT_DIV		0x19
#define	CONFIG			0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	USER_CTRL	    0x6A
#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I	    0x75
#define	MPU6050_Addr	0x68

class MPU6050{
	float offset_ax,offset_ay,offset_az;
	float offset_gx,offset_gy,offset_gz;
public:
	MPU6050(){
		offset_ax=0;
		offset_ay=0;
		offset_az=0;
		offset_gx=0;
		offset_gy=0;
		offset_gz=0;
	}

	void Start();
	void GetRaw(short *ax, short *ay, short *az, short *gx, short *gy, short *gz, short *tem);
	u8 Get(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *tem);
	u8 GetAverage(int num, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *tem);
	u8 Calibration(int num);
	void SetOffSetAccel(float ax, float ay, float az);
	void SetOffSetGyro(float gx, float gy, float gz);
};

extern void _delay(u32 value);

#endif /* MPU6050_H_ */
