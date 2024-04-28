#include "h_bridge.hpp"

int IN1 = 21;
int IN2 = 20;
int IN3 = 19;
int IN4 = 18;

void h_bridge_setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

// limit the pwm value to HBMaxPWM or -HBMaxPWM
int h_bridge_pwm_limit(int pwm)
{
    if (abs(pwm) > HBMaxPWM)
    {
        if (pwm < 0)
        {
            return -HBMaxPWM;
        }
        return HBMaxPWM;
    }
    return pwm;
}

// Set the pines INA or INB pwm value and left the other in zero depending of the sign of the pwm
void h_bridge_set_pwm(int INA, int INB, int pwm)
{
    pwm = h_bridge_pwm_limit(pwm);

    if (pwm > 0)
    {
        analogWrite(INA, 0); 
        analogWrite(INB, pwm);  
    }
    else if (pwm < 0)
    {
        analogWrite(INB, 0); 
        analogWrite(INA, abs(pwm));  
    }
    else
    {
        analogWrite(INB, 0); 
        analogWrite(INA, 0); 
    }
}

// Set the right and left motor pwm values
void h_bridge_pwm(int pwm_r, int pwm_l)
{
    h_bridge_set_pwm(RIGHT, pwm_r);
    h_bridge_set_pwm(LEFT, -pwm_l);
}