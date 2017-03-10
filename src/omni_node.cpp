#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <raspi_gpio/omni.h>
#include <string>

Omni omni;
bool movecmdflag = false;
double motion[3] = {};

void movecmdCallback(const std_msgs::Float32MultiArray& msg)
{
	motion[0] = msg.data[0];
	motion[1] = msg.data[1];
	motion[2] = msg.data[2];
	movecmdflag = true;
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
	node_.param("cmd_topic", cmd_topic_,std::string("/omni_movecmd"));
	ros::Subscriber MovecmdSub = node_.subscribe(cmd_topic_, 1, movecmdCallback);
	ros::Rate looprate(looprate_);

	omni.GpioInit();
	omni.PwmCreateSetup();
	omni.pinModeInputSetup();
	omni.wiringPiISRSetup();
	
	while(ros::ok())
	{
		if(movecmdflag) { 
			omni.movecmd_write(motion[0], motion[1], motion[2]);
			omni.output();
		}
		else {
			omni.stop();
		}
		omni.dispstatus();
		omni.pulseReset();
		movecmdflag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
