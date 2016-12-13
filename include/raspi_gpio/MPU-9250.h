#ifndef MPU_9250_H_
#define MPU_9250_H_

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>

//#define MAG_MODE_POWERDOWN 0x10
#define MAG_MODE_SINGLE    0x11
#define MAG_MODE_8HZ       0x12
#define MAG_MODE_100HZ     0x16
//#define MAG_MODE_TRIGER    0x14
//#define MAG_MODE_SELFTEST  0x18

namespace mpu9250 {

	struct axisData {
		double x;
		double y;
		double z;
	};
	
	int I2CInit(int);
	void StartSensing(int);
	void accelScale(int,int);
	void gyroScale(int,int);
	void MagModeSet(int,int);
	int S2U(int);
	axisData getAccel(int);
	axisData getGyro(int);
	axisData getMag(int);
	void accelOffset(int, int count = 100);
	void gyroOffset(int, int count = 100);
}

#endif
