#include <raspi_gpio/omni.h>
using namespace std;

void PwmCreateSetup(void)
{
	softPwmCreate(MOTOR1A, 0, RANGE);
	softPwmCreate(MOTOR1B, 0, RANGE);
	softPwmCreate(MOTOR2A, 0, RANGE);
	softPwmCreate(MOTOR2B, 0, RANGE);
	softPwmCreate(MOTOR3A, 0, RANGE);
	softPwmCreate(MOTOR3B, 0, RANGE);
}

void pinModeInputSetup(void)
{
	pinMode(SIG1A, INPUT);
	pinMode(SIG1B, INPUT);
	pinMode(SIG2A, INPUT);
	pinMode(SIG2B, INPUT);
	pinMode(SIG3A, INPUT);
	pinMode(SIG3B, INPUT);
}

void wiringPiISRSetup(void)
{
	wiringPiISR(SIG1A, INT_EDGE_BOTH, pin1A_changed);
	wiringPiISR(SIG1B, INT_EDGE_BOTH, pin1B_changed);
	wiringPiISR(SIG2A, INT_EDGE_BOTH, pin2A_changed);
	wiringPiISR(SIG2B, INT_EDGE_BOTH, pin2B_changed);
	wiringPiISR(SIG3A, INT_EDGE_BOTH, pin3A_changed);
	wiringPiISR(SIG3B, INT_EDGE_BOTH, pin3B_changed);
}

void pin1A_changed(void)
{
	if(digitalRead(SIG1A)) {
		if(digitalRead(SIG1B)) pulse[0]--;
		else pulse[0]++;
	}
	else {
		if(digitalRead(SIG1B)) pulse[0]++;
		else pulse[0]--;
	}
}

void pin1B_changed(void)
{
	if(digitalRead(SIG1A)) {
		if(digitalRead(SIG1B)) pulse[0]++;
		else pulse[0]--;
	}
	else {
		if(digitalRead(SIG1B)) pulse[0]--;
		else pulse[0]++;
	}
}

void pin2A_changed(void)
{
	if(digitalRead(SIG2A)) {
		if(digitalRead(SIG2B)) pulse[1]--;
		else pulse[1]++;
	}
	else {
		if(digitalRead(SIG2B)) pulse[1]++;
		else pulse[1]--;
	}
}

void pin2B_changed(void)
{
	if(digitalRead(SIG2A)) {
		if(digitalRead(SIG2B)) pulse[1]++;
		else pulse[1]--;
	}
	else {
		if(digitalRead(SIG2B)) pulse[1]--;
		else pulse[1]++;
	}
}

void pin3A_changed(void)
{
	if(digitalRead(SIG3A)) {
		if(digitalRead(SIG3B)) pulse[2]--;
		else pulse[2]++;
	}
	else {
		if(digitalRead(SIG3B)) pulse[2]++;
		else pulse[2]--;
	}
}

void pin3B_changed(void)
{
	if(digitalRead(SIG3A)) {
		if(digitalRead(SIG3B)) pulse[2]++;
		else pulse[2]--;
	}
	else {
		if(digitalRead(SIG3B)) pulse[2]--;
		else pulse[2]++;
	}
}

void calc_targetpulse(int* targetpulse, double* movecmd)
{
	for(int i=0; i<NOW; i++) {
		targetpulse[i] = MOT2PUL * (movecmd[0]*cos(wrad[i]) + movecmd[1]*sin(wrad[i]) - movecmd[2]);
		//motorout[i] = motorout[i]/3;
		//cout<<"motorout["<<i<<"] = "<<motorout[i]<<endl;
	}
}

void calc_motorout(double* motorout, int* pulse, int* targetpulse)
{
	double pgain = 1.0/30;
	for(int i=0; i<NOW; i++) {
		motorout[i] = pgain * (targetpulse[i]-pulse[i]);
		if(motorout[i]>1) motorout[i] = 1;
		else if(motorout[i]<-1) motorout[i] = -1;
	}
}

void PWMwrite(double* motorout)
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

void dispstatus(double* m_cmd, int* pulse, int* t_pul, double* motor)
{
	cout<<"m_cmd = "<<m_cmd[0]<<", "<<m_cmd[1]<<", "<<m_cmd[2]<<endl;
	cout<<"pulse = "<<pulse[0]<<", "<<pulse[1]<<", "<<pulse[2]<<endl;
	cout<<"t_pul = "<<t_pul[0]<<", "<<t_pul[1]<<", "<<t_pul[2]<<endl;
	cout<<"motor = "<<motor[0]<<", "<<motor[1]<<", "<<motor[2]<<endl;
	cout<<endl;
}

