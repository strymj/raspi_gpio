#include <raspi_gpio/MPU9250.h>
using namespace std;

namespace mpu9250 {

	int I2CInit(int ID)
	{
		int fd = wiringPiI2CSetup(ID);
		if(fd == -1) {
			cout<<"cannot setup I2C"<<endl;
			exit(0);
		}
		else {
			cout<<"I2C init success!"<<endl;
		}
		return fd;
	}

	bool acgystart = false;
	void StartSensing(int fd)
	{
		wiringPiI2CWriteReg8(fd, 0x6B, 0x00);
		wiringPiI2CWriteReg8(fd ,0x37, 0x02); 
		acgystart = true;
		cout<<"Accel and Gyro sensor start."<<endl;
	}

	int magmode = -1;
	void MagModeSet(int fd, int mode)
	{
		wiringPiI2CWriteReg8(fd, 0x0A, mode);
	}

	int ACRANGE = 2;
	void accelScale(int fd, int range)
	{
		int data = 0x00;

		if(range = 2) {
			data = 0x00;
			ACRANGE = range;
		}
		else if(range = 4) {
			data = 0x08;
			ACRANGE = range;
		}
		else if(range = 8) {
			data = 0x10;
			ACRANGE = range;
		}
		else if(range = 16) {
			data = 0x18;
			ACRANGE = range;
		}
		else {
			cout<<"failed to set accel range."<<endl;
			cout<<"usage : accelrange = {2,4,8,16}"<<endl;
		}

		wiringPiI2CWriteReg8(fd, 0x1c, data);
	}

	int GYRANGE = 250;
	void gyroScale(int fd, int range)
	{
		int data = 0x00;

		if(range = 250) {
			data = 0x00;
			GYRANGE = range;
		}
		else if(range = 500) {
			data = 0x08;
			GYRANGE = range;
		}
		else if(range = 1000) {
			data = 0x10;
			GYRANGE = range;
		}
		else if(range = 2000) {
			data = 0x18;
			GYRANGE = range;
		}
		else {
			cout<<"failed to set gyro range."<<endl;
			cout<<"usage : gyrorange = {250,500,1000,2000}"<<endl;
		}

		wiringPiI2CWriteReg8(fd, 0x1b, data);
	}

	int S2U(int num)
	{
		if (num & 0x8000) {   // if num 16bit == 1
			return -1 * ((num ^ 0xFFFF) + 1);
		}
		else {
			return num;
		}
	}

	axisData ACOFFSET = {0.0, 0.0, 0.0};
	axisData getAccel(int fd)
	{
		int XH = wiringPiI2CReadReg8(fd, 0x3b);
		int XL = wiringPiI2CReadReg8(fd, 0x3c);
		int YH = wiringPiI2CReadReg8(fd, 0x3d);
		int YL = wiringPiI2CReadReg8(fd, 0x3e);
		int ZH = wiringPiI2CReadReg8(fd, 0x3f);
		int ZL = wiringPiI2CReadReg8(fd, 0x40);

		axisData ans;
		ans.x = -ACOFFSET.x + S2U(XH << 8 | XL) * ACRANGE / (double)0x8000;
		ans.y = -ACOFFSET.y + S2U(YH << 8 | YL) * ACRANGE / (double)0x8000;
		ans.z = -ACOFFSET.z + S2U(ZH << 8 | ZL) * ACRANGE / (double)0x8000;

		return ans;
	}

	axisData GYOFFSET = {0.0, 0.0, 0.0};
	axisData getGyro(int fd)
	{
		int XH = wiringPiI2CReadReg8(fd, 0x43);
		int XL = wiringPiI2CReadReg8(fd, 0x44);
		int YH = wiringPiI2CReadReg8(fd, 0x45);
		int YL = wiringPiI2CReadReg8(fd, 0x46);
		int ZH = wiringPiI2CReadReg8(fd, 0x47);
		int ZL = wiringPiI2CReadReg8(fd, 0x47);

		axisData ans;
		ans.x = -GYOFFSET.x + S2U(XH << 8 | XL) * GYRANGE / (double)0x8000;
		ans.y = -GYOFFSET.y + S2U(YH << 8 | YL) * GYRANGE / (double)0x8000;
		ans.z = -GYOFFSET.z + S2U(ZH << 8 | ZL) * GYRANGE / (double)0x8000;

		return ans;
	}

	axisData getMag(int fd)
	{
		if(magmode == -1 || magmode == MAG_MODE_SINGLE) {
			wiringPiI2CWriteReg8(fd, 0x0a, MAG_MODE_SINGLE);
		}

		while(!(wiringPiI2CReadReg8(fd, 0x02) & 0x01)) {   // !dataready
			wiringPiI2CReadReg8(fd, 0x02);
			//cout<<"dataready : "<<wiringPiI2CReadReg8(fd, 0x02)<<endl;
			usleep(100);
		}


		int XL = wiringPiI2CReadReg8(fd, 0x03);
		int XH = wiringPiI2CReadReg8(fd, 0x04);
		int YL = wiringPiI2CReadReg8(fd, 0x05);
		int YH = wiringPiI2CReadReg8(fd, 0x06);
		int ZL = wiringPiI2CReadReg8(fd, 0x07);
		int ZH = wiringPiI2CReadReg8(fd, 0x08);
		
		bool ofl = wiringPiI2CReadReg8(fd, 0x09)>>3 & 0x01;
		cout<<"oflow  : "<<ofl<<endl;

		axisData ans;
		ans.x = S2U(XH << 8 | XL);
		ans.y = S2U(YH << 8 | YL);
		ans.z = S2U(ZH << 8 | ZL);

		return ans;
	}

	void accelOffset(int fd, int count)
	{
		axisData offset = {0.0, 0.0, 0.0};
		for(int i=0; i<count; i++) {
			axisData read = getAccel(fd);
			offset.x += read.x;
			offset.y += read.y;
			offset.z += read.z;
			usleep(10000);
		}
		offset.x /= count;
		offset.y /= count;
		offset.z /= count;
		ACOFFSET = offset;
	}

	void gyroOffset(int fd, int count)
	{
		axisData offset = {0.0, 0.0, 0.0};
		for(int i=0; i<count; i++) {
			axisData read = getGyro(fd);
			offset.x += read.x;
			offset.y += read.y;
			offset.z += read.z;
			usleep(10000);
		}
		offset.x /= count;
		offset.y /= count;
		offset.z /= count;
		GYOFFSET = offset;
	}

}
