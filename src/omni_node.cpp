#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
#include <string>

bool movecmdflag = false;
bool markerflag = false;

void movecmdCallback(const std_msgs::Float32MultiArray& msg)
{
	movecmd[0] = msg.data[0];
	movecmd[1] = msg.data[1];
	movecmd[2] = msg.data[2];
	movecmdflag = true;
}

void markerflagCallback(const std_msgs::Bool& msg)
{
	markerflag = msg.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "omni_node");
	ros::NodeHandle node_("~");

	int looprate_;
	double emstop_time_;
	std::string cmd_topic_, flag_topic_;
	node_.param("looprate", looprate_, 30);
	node_.param("emstop_time", emstop_time_, 0.5);
	node_.param("cmd_topic", cmd_topic_,std::string("/position_correction/movecmd"));
	node_.param("flag_topic", flag_topic_,std::string("/picture_joystick/MarkerFlag"));
	ros::Subscriber MovecmdSub = node_.subscribe(cmd_topic_, 1, movecmdCallback);
	ros::Subscriber MarkerFlagSub = node_.subscribe(flag_topic_, 1, markerflagCallback);
	ros::Rate looprate(looprate_);

	if(!GpioInit()) return -1;
	PwmCreateSetup();
	pinModeInputSetup();
	wiringPiISRSetup();

	while(ros::ok())
	{
		if(movecmdflag && markerflag) { 
			calc_targetpulse(targetpulse, movecmd, ratio);
			calc_motorout(motorout, pulse, targetpulse, gain);
		}
		else {
			stop(motorout);
		}
		PWMwrite(motorout);
		dispstatus(movecmd, pulse, targetpulse, motorout);
		pulseReset(pulse);
		movecmdflag = false;
		markerflag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
