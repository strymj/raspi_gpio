#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "wiringPi.h"
#define GPIO18 18
#define RANGE 1024
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
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(200);
	pwmSetRange(RANGE);
	pinMode(GPIO18, PWM_OUTPUT);

	int loopcount = 0;
	while(ros::ok())
	{
		double nowPWM = (sin(M_PI/40*loopcount)+1) * RANGE/2;
		pwmWrite(GPIO18, nowPWM);

		loopcount++;
		looprate.sleep();
	}
	pwmWrite(GPIO18,0);

	return 0;
}