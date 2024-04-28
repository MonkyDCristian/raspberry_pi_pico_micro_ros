#ifndef _H_BRIDGE_H_
#define _H_BRIDGE_H_

#include "Arduino.h"

#define HBMaxPWM 255

#define RIGHT IN1, IN2
#define LEFT IN3, IN4

extern int IN1;
extern int IN2;
extern int IN3;
extern int IN4;

void h_bridge_setup();
int h_bridge_pwm_limit(int pwm);
void h_bridge_set_pwm(int INA, int INB, int pwm);
void h_bridge_pwm(int pwm_r, int pwm_l);

#endif