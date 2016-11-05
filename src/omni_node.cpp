#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
using namespace std;

double movecmd[3] = {0,0,0};

void movecmdCallback(const std_msgs::Float32MultiArray& msg)
{
	movecmd[0] = msg.data[0];
	movecmd[1] = msg.data[1];
	movecmd[2] = msg.data[2];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_;
	ros::Subscriber sub = node_.subscribe("/keyjoy/movecmd", 1, movecmdCallback);
	int looprate_;
	node_.param("looprate", looprate_, 30);
	ros::Rate looprate(looprate_);

	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		return -1;
	}
	else cout<<"setup gpio success!"<<endl;

	PwmCreateSetup();
	pinModeInputSetup();
	wiringPiISRSetup();

	int targetpulse[3] = {0,0,0};
	double motorout[3] = {0,0,0};

	while(ros::ok())
	{
		calc_targetpulse(targetpulse, movecmd);
		calc_motorout(motorout, pulse, targetpulse);
		dispstatus(movecmd, pulse, targetpulse, motorout);
		PWMwrite(motorout);
		pulse[0] = 0;
		pulse[1] = 0;
		pulse[2] = 0;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
