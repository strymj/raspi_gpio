#include <ros/ros.h>
#include <raspi_gpio/MPU-9250.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyro_input");
	ros::NodeHandle node_("~");

	int looprate_, ID_;
	node_.param("looprate", looprate_, 10);
	ros::Rate looprate(looprate_);

	int fd_acgy = mpu9250::I2CInit(0x69);
	mpu9250::StartSensing(fd_acgy);
	int fd_mag = mpu9250::I2CInit(0x0c);
	
	//mpu9250::accelScale(fd_acgy,2);
	//mpu9250::gyroScale(fd_acgy,250);
	//mpu9250::accelOffset(fd_acgy);
	//mpu9250::gyroOffset(fd_acgy);
	mpu9250::MagModeSet(fd_mag, MAG_MODE_100HZ);

	//mpu9250::axisData ac;
	//mpu9250::axisData gy;
	mpu9250::axisData mg;

	cout << fixed << setprecision(3); 

	while(ros::ok())
	{
		//ac = mpu9250::getAccel(fd_acgy);
		//gy = mpu9250::getGyro(fd_acgy);
		mg = mpu9250::getMag(fd_mag);
		//cout<<"accelX : "<<ac.x<<endl;
		//cout<<"accelY : "<<ac.y<<endl;
		//cout<<"accelZ : "<<ac.z<<endl;
		//cout<<"gyroX  : "<<gy.x<<endl;
		//cout<<"gyroY  : "<<gy.y<<endl;
		//cout<<"gyroZ  : "<<gy.z<<endl;
		cout<<"magneX : "<<mg.x<<endl;
		cout<<"magneY : "<<mg.y<<endl;
		cout<<"magneZ : "<<mg.z<<endl;
		cout<<endl;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
