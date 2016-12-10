#include <ros/ros.h>
#include <raspi_gpio/MPU-9250.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyro_input");
	ros::NodeHandle node_("~");

	int looprate_, ID_;
	node_.param("looprate", looprate_, 10);
	node_.param("ID", ID_, 0x69);
	ros::Rate looprate(looprate_);

	int fd = acgy::I2CInit(ID_);
	acgy::StartSensing();
	//acgy::accelScale(fd,2);
	//acgy::gyroScale(fd,250);
	//acgy::accelOffset(fd);
	//acgy::gyroOffset(fd);

	//acgy::axisData ac;
	//acgy::axisData gy;
	acgy::axisData mg;

	cout << fixed << setprecision(3); 

	int reg = 0;
	while(ros::ok())
	{
		//ac = acgy::getAccel(fd);
		//gy = acgy::getGyro(fd);
		mg = acgy::getMag(fd);
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
