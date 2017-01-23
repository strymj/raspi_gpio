#ifndef OMNI_H_
#define OMNI_H_

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <wiringPi.h>
#include <softPwm.h>

#define RANGE 216   // PWM range
#define NOW 3       // Number Of Wheels

// GPIO pin define
#define MOTOR1A 25
#define MOTOR1B 8
#define MOTOR2A 23
#define MOTOR2B 24
#define MOTOR3A 17
#define MOTOR3B 27

// OMRON E6A2-CW3C ENCODER INPUT
//#define SIG1A 6     // OUT B
//#define SIG1B 13    // OUT A
#define SIG1A 10     // OUT B
#define SIG1B 9    // OUT A
#define SIG2A 19
#define SIG2B 26
#define SIG3A 20
#define SIG3B 21

static double MAXPULSE = 3000.0 / 30;   // 3000[pulse/s] / 30[/s]
static double wrad[3] = {0, M_PI*2/3, M_PI*4/3};   // 0, 120, 240 [deg]

double movecmd[3] = {0,0,0};
double pose[3] = {0,0,0};
double sensorvalue[9] = {0,0,0,0,0,0,0,0,0};
int pulse[3] = {0,0,0};
int targetpulse[3] = {0,0,0};
double motorout[3] = {0,0,0};
double gain[3] = {0.008, 0.001, 0.010};   // p, i, d gain
double ratio[2] = {0.8, 0.2};    // ratio move : rotate

void GpioInit(void);
void PwmCreateSetup(void);
void wiringPiISRSetup(void);
void pinModeInputSetup(void);

void pin1A_changed(void);
void pin1B_changed(void);
void pin2A_changed(void);
void pin2B_changed(void);
void pin3A_changed(void);
void pin3B_changed(void);

void stop(double*);
void calc_targetpulse(int*, double*, double*);
void calc_motorout(double*, int*, int*, double*);
void PWMwrite(double*);
void pulseReset(int*);
void dispstatus(double*, int*, int*, double*);

#endif
