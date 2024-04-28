#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "Arduino.h"

// confing encoders Pins
#define EncPinMlA 14
#define EncPinMlB 15
#define EncPinMrA 16
#define EncPinMrB 17

// variables
extern volatile int mr_pos, ml_pos;
extern int last_mr_pos, last_ml_pos;
extern int mr_vel, ml_vel;
extern long enc_last_time;

void encoder_setup();
void enc_mr_a_cb();
void enc_mr_b_cb();
void enc_ml_a_cb();
void enc_ml_b_cb();
void enc_measure_vel();

#endif