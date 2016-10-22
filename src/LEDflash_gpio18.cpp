#include <ros/ros.h>
#include <iostream>
#include "wiringPi.h"
#define GPIO18 18
#define FLASHRATE 3
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LEDflash_gpio18");
	ros::NodeHandle node_;
	ros::Rate looprate(10);

	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		return -1;
	}
	pinMode(GPIO18, OUTPUT);

	int loopcount = 0;
	while(ros::ok())
	{
		if(loopcount/FLASHRATE%2) digitalWrite(GPIO18,1);
		else digitalWrite(GPIO18,0);
	
		loopcount++;
		looprate.sleep();
	}

	digitalWrite(GPIO18,0);
	return 0;
}
