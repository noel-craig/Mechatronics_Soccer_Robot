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

void Chassis::rotateAbout(int speed, int wheel) {
    // 1  FR
    if (wheel == 0){
        front_left.setSpeed(speed);
        front_right.setSpeed(0);
        back_left.setSpeed(-speed);
        back_right.setSpeed(speed);      
    }
    // 2 FL
    else if (wheel == 1){
        front_left.setSpeed(0);
        front_right.setSpeed(-speed);
        back_left.setSpeed(-speed);
        back_right.setSpeed(speed);
    }
    // 3 RR
    else if (wheel == 2){
        front_left.setSpeed(speed);
        front_right.setSpeed(-speed);
        back_left.setSpeed(-speed);
        back_right.setSpeed(0);
    }
    // 4 RL
    else if (wheel == 3){
        front_left.setSpeed(speed);
        front_right.setSpeed(-speed);
        back_left.setSpeed(0);
        back_right.setSpeed(-speed);
    } 
    else{
        front_left.setSpeed(0);
        front_right.setSpeed(0);
        back_left.setSpeed(0);
        back_right.setSpeed(0); 
    }

}

void Chassis::angleDriveRotate(int speed, int angle, int rotation_speed) {
    // float rat = rotation_speed / (speed+rotation_speed);
    // translation part
    float fl_br = sin(radians(angle+45))*speed; // * (1 - rat);
    float fr_bl = sin(radians(angle+135))*speed; // * (1 - rat);
    // rotation_speed = rotation_speed * rat;

    // add rotation straight on
    float fl = fl_br + rotation_speed;
    float br = fl_br - rotation_speed;
    float fr = fr_bl - rotation_speed;
    float bl = fr_bl + rotation_speed;

    // find the biggest wheel command (ignores sign)
    float largest = fabs(fl);
    if (fabs(fr)>largest) largest = fabs(fr);
    if (fabs(bl)>largest) largest = fabs(bl);
    if (fabs(br)>largest) largest = fabs(br);

    // if any wheel is over 255, shrink all four by the same ratio
    if (largest >255.0) {
        float scale = 255.0/largest;
        fl *= scale;
        fr *= scale;
        bl *= scale;
        br *= scale;
    }
    // send to wheels
    front_left.setSpeed(fl);
    back_right.setSpeed(br);
    front_right.setSpeed(fr);
    back_left.setSpeed(bl);
}