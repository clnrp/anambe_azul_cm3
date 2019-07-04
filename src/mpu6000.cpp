/*
 * mpu6050.cpp
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#include "mpu6000.h"

void MPU6050::Start()
{
	u8 buffer[20];
	//SPI_WriteByte(USER_CTRL, 0x08);

	I2C_Read2(MPU6050_Addr, buffer, WHO_AM_I, 1);
	_delay(1000);

	I2C_ByteWrite(MPU6050_Addr,PWR_MGMT_1, 0x01);
	_delay(1000);
	I2C_ByteWrite(MPU6050_Addr,GYRO_CONFIG, 0x00);   // Set Gyro Full Scale Range to +-250deg/s
	_delay(1000);
	I2C_ByteWrite(MPU6050_Addr,ACCEL_CONFIG, 0x00);  // Set Accelerometer Full Scale Range to +-2g
}

u8 MPU6050::Get(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *tem){
	u8 buffer[15];
	short tmp;

	//SPI_WriteByte(USER_CTRL, 0x08);
	//buffer[0] = SPI_ReadByte(ACCEL_XOUT_H);
	if(!I2C_Read2(MPU6050_Addr, buffer, ACCEL_XOUT_H, 14))
		return 0x00;
    tmp = (buffer[0]<<8) | buffer[1];
	*ax = (float)tmp / 16384.0*9.80665 - offset_ax;
    tmp = (buffer[2]<<8) | buffer[3];
	*ay = (float)tmp / 16384.0*9.80665 - offset_ay;
    tmp = (buffer[4]<<8) | buffer[5];
	*az = (float)tmp / 16384.0*9.80665 - offset_az;

	tmp = (buffer[6]<<8) | buffer[7];
	*tem  = (float)tmp / 340 + 36.53;
	tmp = (buffer[8]<<8) | buffer[9];
	*gx = (float)tmp / 131.0 - offset_gx;
	tmp = (buffer[10]<<8) | buffer[11];
	*gy = (float)tmp / 131.0 - offset_gy;
	tmp = (buffer[12]<<8) | buffer[13];
	*gz = (float)tmp / 131.0 - offset_gz; // graus/s
	return 0x01;
}

u8 MPU6050::GetAverage(int num, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *tem){
	u8 buffer[15];
	float m_ax=0,m_ay=0,m_az=0;
	float m_gx=0,m_gy=0,m_gz=0;
	float m_tem=0;
	short tmp;

	for(int i=0; i<num; i++ ){
		if(!I2C_Read2(MPU6050_Addr, buffer, ACCEL_XOUT_H, 14))
			return 0x00;
		_delay(100);
		tmp = (buffer[0]<<8) | buffer[1];
		m_ax += (float)tmp / 16384.0*9.80665 - offset_ax;
		tmp = (buffer[2]<<8) | buffer[3];
		m_ay += (float)tmp / 16384.0*9.80665 - offset_ay;
		tmp = (buffer[4]<<8) | buffer[5];
		m_az += (float)tmp / 16384.0*9.80665 - offset_az;

		tmp = (buffer[6]<<8) | buffer[7];
		m_tem  += (float)tmp / 340 + 36.53;
		tmp = (buffer[8]<<8) | buffer[9];
		m_gx += (float)tmp / 131.0 - offset_gx;
		tmp = (buffer[10]<<8) | buffer[11];
		m_gy += (float)tmp / 131.0 - offset_gy;
		tmp = (buffer[12]<<8) | buffer[13];
		m_gz += (float)tmp / 131.0 - offset_gz; // graus/s
	}
	*ax=m_ax/num;
	*ay=m_ay/num;
	*az=m_az/num;
	*gx=m_gx/num;
	*gy=m_gy/num;
	*gz=m_gz/num;
	*tem=m_tem/num;
	return 0x01;
}

u8 MPU6050::Calibration(int num)
{
    float ax=0,ay=0,az=0;
    float gx=0,gy=0,gz=0;
    double m_ax=0,m_ay=0,m_az=0;
    double m_gx=0,m_gy=0,m_gz=0;
    float tem;
	for (int i=0; i<num; i++){
		if(!this->Get(&ax,&ay,&az,&gx,&gy,&gz,&tem))
			return 0x00;
		_delay(1000);
		m_ax+=ax;
		m_ay+=ay;
		m_az+=az;
		m_gx+=gx;
		m_gy+=gy;
		m_gz+=gz;
	}

	offset_ax = (float)m_ax/num;
	offset_ay = (float)m_ay/num;
	offset_az = (float)m_az/num-9.80665;
	offset_gx = (float)m_gx/num;
	offset_gy = (float)m_gy/num;
	offset_gz = (float)m_gz/num;
	return 0x01;
}

void MPU6050::SetOffSetAccel(float ax, float ay, float az)
{
	offset_ax = ax;
	offset_ay = ay;
	offset_az = az;
}

void MPU6050::SetOffSetGyro(float gx, float gy, float gz)
{
	offset_gx = gx;
	offset_gy = gy;
	offset_gz = gz;
}
