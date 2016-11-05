#ifndef OMNI_H_
#define OMNI_H_

#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE 216   // PWM range
#define NOW 3       // Number Of Wheels

#define MOTOR1A 4
#define MOTOR1B 3
#define MOTOR2A 23
#define MOTOR2B 24
#define MOTOR3A 11
#define MOTOR3B 9

#define SIG1A 12
#define SIG1B 6
#define SIG2A 26
#define SIG2B 19
#define SIG3A 20
#define SIG3B 21

#define MOT2PUL 100   // motor2pulse 100[pulse] / 1

double wrad[3] = {0, M_PI*2/3, M_PI*4/3};
int pulse[3] = {0,0,0};

void PwmCreateSetup(void);

void wiringPiISRSetup(void);

void pinModeInputSetup(void);

void pin1A_changed(void);
void pin1B_changed(void);
void pin2A_changed(void);
void pin2B_changed(void);
void pin3A_changed(void);
void pin3B_changed(void);

void calc_targetpulse(int*, double*);

void calc_motorout(double*, int*, int*);

void PWMwrite(double*);

void dispstatus(double*, int*, int*, double*);

#endif
