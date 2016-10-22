#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "wiringPi.h"
#define GPIO17 17
#define GPIO18 18
#define UPRATE 60
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
	//pinMode(GPIO17, PWM_OUTPUT);
	pinMode(GPIO18, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(200);
	pwmSetRange(1024);

	int loopcount = 0;
	while(ros::ok())
	{
		double nowPWM = sin(M_PI/UPRATE*loopcount) * 1024;
		cout<<nowPWM<<endl;
		if(nowPWM>=0) {
			//pwmWrite(GPIO17, 0);
			pwmWrite(GPIO18, (int)nowPWM);
		}
		else {
			//pwmWrite(GPIO17, (int)-nowPWM);
			pwmWrite(GPIO18, 0);
		}

		loopcount++;
		looprate.sleep();
	}
	pwmWrite(GPIO17,0);
	pwmWrite(GPIO18,0);
	
	return 0;
}
