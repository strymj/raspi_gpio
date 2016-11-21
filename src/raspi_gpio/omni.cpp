#include <raspi_gpio/omni.h>
using namespace std;

int GpioInit(void)
{
	if(wiringPiSetupGpio() == -1) {
		cout<<"cannot setup gpio."<<endl;
		return 0;
	}
	else {
		cout<<"gpio init success!"<<endl;
	}
	return 1;
}

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

void emergency_stop(int* targetpulse, double pasttime, double emstop_time)
{
	cout<<"pasttime = "<<pasttime<<endl;
	if(pasttime > emstop_time) {
		targetpulse[0] = 0;
		targetpulse[1] = 0;
		targetpulse[2] = 0;
	}
}

void calc_targetpulse(int* targetpulse, double* movecmd, double* ratio)
{
	double norm = sqrt(movecmd[0]*movecmd[0] + movecmd[1]*movecmd[1]);
	if(norm>1) {
		movecmd[0] /= norm;
		movecmd[1] /= norm;
	}
	for(int i=0; i<NOW; i++) {
		double pulseMove = ratio[0] * (movecmd[0]*cos(wrad[i]) + movecmd[1]*sin(wrad[i]));
		double pulseRotate = -ratio[1] * movecmd[2];
		targetpulse[i] = MAXPULSE * (pulseMove + pulseRotate);
	}
}

void calc_motorout(double* motorout, int* pulse, int* target, double* gain)
{
	static int Past[3] = {0,0,0};
	static int Diff[3] = {0,0,0};
	static int PastDiff[3] = {0,0,0};
	static int Dist[3] = {0,0,0};
	for(int i=0; i<NOW; i++) {
		Diff[i] = pulse[i] - target[i];
		Dist[i] += Diff[i];
		motorout[i] =
			- gain[0] * (pulse[i]-target[i])
			- gain[1] * Dist[i]
			- gain[2] * (Diff[i] - PastDiff[i]);
		PastDiff[i] = Diff[i]; 
		if(motorout[i]>1) motorout[i] = 1;
		else if(motorout[i]<-1) motorout[i] = -1;
	}
}

void PWMwrite(double* motorout)
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

void pulseReset(int* pulse)
{
	pulse[0] = 0;
	pulse[1] = 0;
	pulse[2] = 0;
}

void dispstatus(double* m_cmd, int* pulse, int* t_pul, double* motor)
{
	cout<<"m_cmd = "<<m_cmd[0]<<", "<<m_cmd[1]<<", "<<m_cmd[2]<<endl;
	cout<<"pulse = "<<pulse[0]<<", "<<pulse[1]<<", "<<pulse[2]<<endl;
	cout<<"t_pul = "<<t_pul[0]<<", "<<t_pul[1]<<", "<<t_pul[2]<<endl;
	cout<<"motor = "<<motor[0]<<", "<<motor[1]<<", "<<motor[2]<<endl;
	cout<<endl;
}

