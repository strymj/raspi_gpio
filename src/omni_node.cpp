#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
#include <string>

int submiss_count = 0;

void movecmdCallback(const std_msgs::Float32MultiArray& msg)
{
	movecmd[0] = msg.data[0];
	movecmd[1] = msg.data[1];
	movecmd[2] = msg.data[2];
	submiss_count = 0;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_("~");

	int looprate_;
	double emstop_time_;
	std::string cmd_topic_;
	node_.param("looprate", looprate_, 30);
	node_.param("emstop_time", emstop_time_, 0.5);
	node_.param("cmd_topic", cmd_topic_,std::string("movecmd"));
	ros::Subscriber sub = node_.subscribe(cmd_topic_, 1, movecmdCallback);
	ros::Rate looprate(looprate_);

	if(!GpioInit()) return -1;
	PwmCreateSetup();
	pinModeInputSetup();
	wiringPiISRSetup();

	while(ros::ok())
	{
		calc_targetpulse(targetpulse, movecmd, ratio);
		emergency_stop(targetpulse, (double)submiss_count/looprate_, emstop_time_);
		calc_motorout(motorout, pulse, targetpulse, gain);
		dispstatus(movecmd, pulse, targetpulse, motorout);
		PWMwrite(motorout);
		pulseReset(pulse);
		ros::spinOnce();
		submiss_count++;
		looprate.sleep();
	}

	return 0;
}
