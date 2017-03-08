#include <raspi_gpio/omni.h>
using namespace std;

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

void wiringPiISRSetup() {
	wiringPiISR(SIG1A, INT_EDGE_BOTH, pin1A_changed);
	wiringPiISR(SIG1B, INT_EDGE_BOTH, pin1B_changed);
	wiringPiISR(SIG2A, INT_EDGE_BOTH, pin2A_changed);
	wiringPiISR(SIG2B, INT_EDGE_BOTH, pin2B_changed);
	wiringPiISR(SIG3A, INT_EDGE_BOTH, pin3A_changed);
	wiringPiISR(SIG3B, INT_EDGE_BOTH, pin3B_changed);
}


Omni::Omni()
{
	// movecmd initialize
	movecmd[0] = 0.0;
	movecmd[1] = 0.0;
	movecmd[2] = 0.0;

	// p,i,d gain initialize
	gain[0] = 0.008;   // p gain
	gain[1] = 0.002;   // i gain
	gain[2] = 0.013;   // d gain

	// translation rotation ratio initialize
	ratio[0] = 0.8;    // move
	ratio[1] = 0.2;    // rotate
}

void Omni::GpioInit(void)
{
	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		exit(0);
	}
	else {
		cout<<"gpio init success!"<<endl;
	}
}

void Omni::PwmCreateSetup(void)
{
	softPwmCreate(MOTOR1A, 0, RANGE);
	softPwmCreate(MOTOR1B, 0, RANGE);
	softPwmCreate(MOTOR2A, 0, RANGE);
	softPwmCreate(MOTOR2B, 0, RANGE);
	softPwmCreate(MOTOR3A, 0, RANGE);
	softPwmCreate(MOTOR3B, 0, RANGE);
}

void Omni::pinModeInputSetup(void)
{
	pinMode(SIG1A, INPUT);
	pinMode(SIG1B, INPUT);
	pinMode(SIG2A, INPUT);
	pinMode(SIG2B, INPUT);
	pinMode(SIG3A, INPUT);
	pinMode(SIG3B, INPUT);
}

void Omni::set_PID_Gain(double p, double i, double d)
{
	gain[0] = p;
	gain[1] = i;
	gain[2] = d;
}

void Omni::set_ratio(double motion, double rotate)
{
	ratio[0] = motion;
	ratio[1] = rotate;
}

void Omni::movecmd_write(double x, double y, double t)
{
	movecmd[0] = x;
	movecmd[1] = y;
	movecmd[2] = t;

	for(int i=0; i<3; i++) {
		if(movecmd[i] > 1) movecmd[i] = 1;
		if(movecmd[i] <-1) movecmd[i] =-1;
	}
}

void Omni::calc_targetpulse()
{
	double norm = sqrt(movecmd[0]*movecmd[0] + movecmd[1]*movecmd[1]);
	if(norm>1) {
		movecmd[0] /= norm;
		movecmd[1] /= norm;
	}
	for(int i=0; i<3; i++) {
		double pulseMove = ratio[0]/(ratio[0]+ratio[1]) * (movecmd[0]*sin(wrad[i]) + movecmd[1]*cos(wrad[i]));
		double pulseRotate = -ratio[1]/(ratio[0]+ratio[1]) * movecmd[2];
		targetpulse[i] = MAXPULSE * (pulseMove + pulseRotate);
	}
}

void Omni::calc_motorout()
{
	static int Past[3] = {0,0,0};
	static int Diff[3] = {0,0,0};
	static int PastDiff[3] = {0,0,0};
	static int Dist[3] = {0,0,0};
	for(int i=0; i<NOW; i++) {
		Diff[i] = pulse[i] - targetpulse[i];
		Dist[i] += Diff[i];
		motorout[i] =
			- gain[0] * (pulse[i]-targetpulse[i])
			- gain[1] * Dist[i]
			- gain[2] * (Diff[i] - PastDiff[i]);
		PastDiff[i] = Diff[i]; 
		if(motorout[i]>1) motorout[i] = 1;
		else if(motorout[i]<-1) motorout[i] = -1;
	}
}

void Omni::PWMwrite()
{
	if(motorout[0]>=0) {
		softPwmWrite(MOTOR1A, motorout[0] * RANGE);
		softPwmWrite(MOTOR1B, 0);
	} else {
		softPwmWrite(MOTOR1A, 0);
		softPwmWrite(MOTOR1B, -motorout[0] * RANGE);
	}

	if(motorout[1]>=0) {
		softPwmWrite(MOTOR2A, motorout[1] * RANGE);
		softPwmWrite(MOTOR2B, 0);
	} else {
		softPwmWrite(MOTOR2A, 0);
		softPwmWrite(MOTOR2B, -motorout[1] * RANGE);
	}

	if(motorout[2]>=0) {
		softPwmWrite(MOTOR3A, motorout[2] * RANGE);
		softPwmWrite(MOTOR3B, 0);
	} else {
		softPwmWrite(MOTOR3A, 0);
		softPwmWrite(MOTOR3B, -motorout[2] * RANGE);
	}
}

void Omni::output()
{
	calc_targetpulse();
	calc_motorout();
	PWMwrite();
}

void Omni::stop()
{
	motorout[0] = 0.0;
	motorout[1] = 0.0;
	motorout[2] = 0.0;
	PWMwrite();
}

void Omni::dispstatus()
{
	cout<<"m_cmd = "<<movecmd[0]<<", "<<movecmd[1]<<", "<<movecmd[2]<<endl;
	cout<<"pulse = "<<pulse[0]<<", "<<pulse[1]<<", "<<pulse[2]<<endl;
	cout<<"t_pul = "<<targetpulse[0]<<", "<<targetpulse[1]<<", "<<targetpulse[2]<<endl;
	cout<<"motor = "<<motorout[0]<<", "<<motorout[1]<<", "<<motorout[2]<<endl;
	cout<<endl;
}

void Omni::pulseReset()
{
	pulse[0] = 0;
	pulse[1] = 0;
	pulse[2] = 0;
}

