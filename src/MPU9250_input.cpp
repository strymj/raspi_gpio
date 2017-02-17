#include <ros/ros.h>
#include <raspi_gpio/omni.h>
#include <raspi_gpio/MPU9250.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
using namespace std;

// tact switch setup
#define TACTSW 5

Omni omni;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyro_input");
	ros::NodeHandle node_("~");

	int looprate_, ID_;
	node_.param("looprate", looprate_, 30);
	ros::Publisher TactSwitchPub = node_.advertise<std_msgs::Bool>("tactswitch",1);
	ros::Publisher MPU9250Pub = node_.advertise<std_msgs::Float32MultiArray>("sensordata",1);
	ros::Rate looprate(looprate_);

	omni.GpioInit();
	pinMode(TACTSW, INPUT);

	int fd_acgy = mpu9250::I2CInit(0x69);
	mpu9250::StartSensing(fd_acgy);
	int fd_mag = mpu9250::I2CInit(0x0c);
	
	mpu9250::accelScale(fd_acgy,2);
	mpu9250::gyroScale(fd_acgy,250);
	//mpu9250::accelOffset(fd_acgy);
	//mpu9250::gyroOffset(fd_acgy);
	mpu9250::MagModeSet(fd_mag, MAG_MODE_100HZ);

	bool tactswitch;
	mpu9250::axisData ac;
	mpu9250::axisData gy;
	mpu9250::axisData mg;

	cout << fixed << setprecision(3); 

	while(ros::ok())
	{
		tactswitch = digitalRead(TACTSW);
		ac = mpu9250::getAccel(fd_acgy);
		gy = mpu9250::getGyro(fd_acgy);
		mg = mpu9250::getMag(fd_mag);
		
		cout<<"accelX : "<<ac.x<<endl;
		cout<<"accelY : "<<ac.y<<endl;
		cout<<"accelZ : "<<ac.z<<endl;
		cout<<"gyroX  : "<<gy.x<<endl;
		cout<<"gyroY  : "<<gy.y<<endl;
		cout<<"gyroZ  : "<<gy.z<<endl;
		cout<<"magneX : "<<mg.x<<endl;
		cout<<"magneY : "<<mg.y<<endl;
		cout<<"magneZ : "<<mg.z<<endl;
		cout<<endl;
	
		std_msgs::Bool tactsw_msg;
		tactsw_msg.data = tactswitch;
		TactSwitchPub.publish(tactsw_msg);
		
		std_msgs::Float32MultiArray MPU9250_msg;
		MPU9250_msg.data.push_back(ac.x);
		MPU9250_msg.data.push_back(ac.y);
		MPU9250_msg.data.push_back(ac.z);
		MPU9250_msg.data.push_back(gy.x);
		MPU9250_msg.data.push_back(gy.y);
		MPU9250_msg.data.push_back(gy.z);
		MPU9250_msg.data.push_back(mg.x);
		MPU9250_msg.data.push_back(mg.y);
		MPU9250_msg.data.push_back(mg.z);
		MPU9250Pub.publish(MPU9250_msg);
		
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
