#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include "wiringPi.h"
#include "softPwm.h"
#define MOTOR1A 17
#define MOTOR1B 18
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

	softPwmCreate(MOTOR1A, 0, RANGE);
	softPwmCreate(MOTOR1B, 0, RANGE);

	int loopcount = 0;
	while(ros::ok())
	{
		double nowPWM = sin(M_PI/40*loopcount) * RANGE;
		if(nowPWM >= 0) {
			softPwmWrite(MOTOR1A, 0);
			softPwmWrite(MOTOR1B, nowPWM);
		} else {
			softPwmWrite(MOTOR1A, -nowPWM);
			softPwmWrite(MOTOR1B, 0);

		}

		loopcount++;
		looprate.sleep();
	}
	softPwmWrite(MOTOR1A,0);
	softPwmWrite(MOTOR1B,0);

	return 0;
}
