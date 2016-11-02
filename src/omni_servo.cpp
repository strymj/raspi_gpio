#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <cmath>
#include "wiringPi.h"
#include "softPwm.h"
#define MOTOR1A 4
#define MOTOR1B 3
#define MOTOR2A 23
#define MOTOR2B 24
#define MOTOR3A 11
#define MOTOR3B 9
#define RANGE 216 
using namespace std;

double wrad[3] = {0, M_PI*2/3, M_PI*4/3};
double movecmd[3] = {0,0,0};
bool subscribed = false;
bool LED = false;

void movecmdCallback(const std_msgs::Float32MultiArray &msg)
{
	movecmd[0] = msg.data[0];
	movecmd[1] = msg.data[1];
	movecmd[2] = msg.data[2];
	cout<<"subscribed  data : "<<msg.data[0]<<","<<msg.data[1]<<","<<msg.data[2]<<endl;
	subscribed = true;
	LED = !LED;
}


void calc_motorout(double motorout[], double movecmd[])
{
	for(int i=0; i<3; i++) {
		motorout[i] = movecmd[0]*cos(wrad[i]) + movecmd[1]*sin(wrad[i]) + movecmd[2];
		//motorout[i] = motorout[i]/3;
		cout<<"motorout["<<i<<"] = "<<motorout[i]<<endl;
	}
}


void PWMwrite(double motorout[])
{
	motorout[0] = motorout[0] * RANGE;
	motorout[1] = motorout[1] * RANGE;
	motorout[2] = motorout[2] * RANGE;

	if(motorout[0]>=0) {
		softPwmWrite(MOTOR1A, motorout[0]);
		softPwmWrite(MOTOR1B, 0);
	} else {
		softPwmWrite(MOTOR1A, 0);
		softPwmWrite(MOTOR1B, -motorout[0]);
	}

	if(motorout[1]>=0) {
		softPwmWrite(MOTOR2A, motorout[1]);
		softPwmWrite(MOTOR2B, 0);
	} else {
		softPwmWrite(MOTOR2A, 0);
		softPwmWrite(MOTOR2B, -motorout[1]);
	}

	if(motorout[2]>=0) {
		softPwmWrite(MOTOR3A, motorout[2]);
		softPwmWrite(MOTOR3B, 0);
	} else {
		softPwmWrite(MOTOR3A, 0);
		softPwmWrite(MOTOR3B, -motorout[2]);
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_LED");
	ros::NodeHandle node_;
	ros::Subscriber sub = node_.subscribe("/keyjoy/movecmd", 1, movecmdCallback);
	ros::Rate looprate(10);

	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		return -1;
	} else cout<<"setup gpio success."<<endl;

	softPwmCreate(MOTOR1A, 0, RANGE);
	softPwmCreate(MOTOR1B, 0, RANGE);
	softPwmCreate(MOTOR2A, 0, RANGE);
	softPwmCreate(MOTOR2B, 0, RANGE);
	softPwmCreate(MOTOR3A, 0, RANGE);
	softPwmCreate(MOTOR3B, 0, RANGE);

	double motorout[3];

	while(ros::ok())
	{
		if(subscribed) {
			calc_motorout(motorout,movecmd);
			PWMwrite(motorout);
		}
		subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}

	softPwmWrite(MOTOR1A, 0);
	softPwmWrite(MOTOR1B, 0);
	softPwmWrite(MOTOR2A, 0);
	softPwmWrite(MOTOR2B, 0);
	softPwmWrite(MOTOR3A, 0);
	softPwmWrite(MOTOR3B, 0);

	return 0;
}
