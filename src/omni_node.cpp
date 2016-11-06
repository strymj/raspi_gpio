#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>

void movecmdCallback(const std_msgs::Float32MultiArray& msg)
{
	movecmd[0] = msg.data[0];
	movecmd[1] = msg.data[1];
	movecmd[2] = msg.data[2];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_("~");
	ros::Subscriber sub = node_.subscribe("/keyjoy/movecmd", 1, movecmdCallback);
	ros::Rate looprate(30);

	if(!GpioInit()) return -1;
	PwmCreateSetup();
	pinModeInputSetup();
	wiringPiISRSetup();

	while(ros::ok())
	{
		calc_targetpulse(targetpulse, movecmd, ratio);
		calc_motorout(motorout, pulse, targetpulse, gain);
		dispstatus(movecmd, pulse, targetpulse, motorout);
		PWMwrite(motorout);
		pulseReset(pulse);
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
