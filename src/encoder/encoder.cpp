#include "encoder.hpp"

// variables
volatile int mr_pos = 0.0, ml_pos = 0.0;
int last_mr_pos = 0, last_ml_pos = 0;
int mr_vel = 0, ml_vel = 0;
long enc_last_time = 0;

void encoder_setup(){
    pinMode(EncPinMrA, INPUT_PULLUP);
    pinMode(EncPinMrB, INPUT_PULLUP);
    pinMode(EncPinMlA, INPUT_PULLUP);
    pinMode(EncPinMlB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EncPinMrA), enc_mr_a_cb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncPinMrB), enc_mr_b_cb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncPinMlA), enc_ml_a_cb, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncPinMlB), enc_ml_b_cb, CHANGE);
}

void enc_mr_a_cb(){
    if (digitalRead(EncPinMrA)!=digitalRead(EncPinMrB)) { mr_pos++;}
    else { mr_pos--;}
}

void enc_mr_b_cb(){
    if (digitalRead(EncPinMrA)==digitalRead(EncPinMrB)) { mr_pos++;}
    else { mr_pos--;}
}

void enc_ml_a_cb(){
    if (digitalRead(EncPinMlA)==digitalRead(EncPinMlB)) { ml_pos++;} // fordward
    else { ml_pos--;} // backward
}

void enc_ml_b_cb(){
    if (digitalRead(EncPinMlA)!=digitalRead(EncPinMlB)) { ml_pos++;}
    else { ml_pos--;}
}

void enc_measure_vel()
{
    mr_vel = mr_pos;
    mr_pos = 0;

    ml_vel = ml_pos;
    ml_pos = 0;
}