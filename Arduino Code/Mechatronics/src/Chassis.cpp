#include "Chassis.h"

Chassis::Chassis(const Wheel& fl, const Wheel& fr, const Wheel& bl, const Wheel& br):
    front_right(fr),
    front_left(fl),
    back_right(br),
    back_left(bl)
    {
    }

void Chassis::init() {
    front_right.init();
    front_left.init();
    back_right.init();
    back_left.init();
}

void Chassis::forward(int speed) {
    front_right.forward(speed);
    front_left.forward(speed);
    back_right.forward(speed);
    back_left.forward(speed);
}

void Chassis::reverse(int speed) {
    front_right.reverse(speed);
    front_left.reverse(speed);
    back_right.reverse(speed);
    back_left.reverse(speed);
}

void Chassis::stop() {
    front_right.stop();
    front_left.stop();
    back_right.stop();
    back_left.stop();
}

void Chassis::angleDrive(int speed, int angle) {
    float fl_br_speed = sin(radians(angle+45))*speed;
    float fr_bl_speed = sin(radians(angle+135))*speed;
    front_left.setSpeed(fl_br_speed);
    back_right.setSpeed(fl_br_speed);
    front_right.setSpeed(fr_bl_speed);
    back_left.setSpeed(fr_bl_speed);
}

void Chassis::rotate(int speed) {
    front_right.forward(speed);
    front_left.reverse(speed);
    back_right.forward(speed);
    back_left.reverse(speed);
}

void Chassis::angleDriveRotate(int speed, int angle, int rotation_speed) {
    float fl_br_speed = sin(radians(angle+45))*speed;
    float fr_bl_speed = sin(radians(angle+135))*speed;
    front_left.setSpeed(fl_br_speed + rotation_speed);
    back_right.setSpeed(fl_br_speed - rotation_speed);
    front_right.setSpeed(fr_bl_speed - rotation_speed);
    back_left.setSpeed(fr_bl_speed + rotation_speed);
}