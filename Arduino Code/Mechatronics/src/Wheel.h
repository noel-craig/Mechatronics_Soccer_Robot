#include <Arduino.h>
#ifndef WHEEL_H
#define WHEEL_H

class Wheel {
  public:
    Wheel(int pwm_pin, int in1_pin, int in2_pin, bool dir_invert = false, float correction_factor = 1.0);
    void init();
    void setSpeed(int speed);
    void forward(int speed);
    void reverse(int speed);
    void stop();

  private:
    int pwm; // PWM Pin for speed control
    int in_1; // direction control pin 1
    int in_2; // direction control pin 2
    bool _reverse; // whether the wheel is reversed 
    float _correction_factor; // correction factor for speed adjustment
};

#endif