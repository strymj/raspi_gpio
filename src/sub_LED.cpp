#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include "wiringPi.h"
#define GPIO18 18
#define FLASHRATE 1
using namespace std;

int subdata = 0;
bool subscribed = false;
bool LED = false;

void subdataCallback(const std_msgs::Int32 &msg)
{
	subdata = msg.data;
	subscribed = true;
}

void proccess(int data)
{
	cout<<"data : "<<data<<endl;
	digitalWrite(GPIO18, data%2);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_LED");
	ros::NodeHandle node_;
	ros::Subscriber sub = node_.subscribe("/data", 1, subdataCallback);
	ros::Rate looprate(10);

	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		return -1;
	}
	else cout<<"setup gpio success."<<endl;

	pinMode(GPIO18, OUTPUT);

	while(ros::ok())
	{
		if(subscribed) {
			proccess(subdata);
		}
		subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}

	digitalWrite(GPIO18,0);
	return 0;
}
