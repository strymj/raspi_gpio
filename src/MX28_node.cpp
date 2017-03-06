#include <ros/ros.h>
#include <raspi_gpio/MX28.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "MX28_node");
	ros::NodeHandle node_("~");

	int looprate_;
	node_.param("looprate", looprate_, 30);
	ros::Rate looprate(looprate_);
	MX28 mx28;
	mx28.portOpen();

	while(ros::ok())
	{
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
