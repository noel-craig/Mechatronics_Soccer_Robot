#ifndef CHASSIS_H
#define CHASSIS_H

#include "Wheel.h"
#include <Arduino.h>
#include <cmath>

class Chassis {
  public:
    Chassis(Wheel fl, Wheel fr, Wheel bl, Wheel br);

    void init();

    void forward(int speed);

    void reverse(int speed);

    void stop();

    void angleDrive(int speed, int angle);

    void rotate(int speed);
    
  private:
    Wheel front_right;
    Wheel front_left;
    Wheel back_right;
    Wheel back_left;
};

#endif