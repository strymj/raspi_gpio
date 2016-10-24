#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "wiringPi.h"
#include "softPwm.h"
#define GPIO17 17
#define GPIO18 18
#define RANGE 512
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

	softPwmCreate(GPIO17, 0, RANGE);
	softPwmCreate(GPIO18, 0, RANGE);

	int loopcount = 0;
	while(ros::ok())
	{
		double nowPWM = sin(M_PI/40*loopcount) * RANGE;
		if(nowPWM >= 0) {
			softPwmWrite(GPIO17, 0);
			softPwmWrite(GPIO18, nowPWM);
		} else {
			softPwmWrite(GPIO17, -nowPWM);
			softPwmWrite(GPIO18, 0);

		}

		loopcount++;
		looprate.sleep();
	}
	softPwmWrite(GPIO17,0);
	softPwmWrite(GPIO18,0);

	return 0;
}
