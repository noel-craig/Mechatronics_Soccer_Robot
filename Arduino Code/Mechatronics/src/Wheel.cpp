#include "Wheel.h"

Wheel::Wheel(int pwm_pin, int in1_pin, int in2_pin, bool dir_invert, float correction_factor) {
    pwm = pwm_pin;
    in_1 = in1_pin;
    in_2 = in2_pin;
    _reverse = dir_invert;
    _correction_factor = correction_factor;
}

void Wheel::init() {
    pinMode(pwm, OUTPUT);
    pinMode(in_1, OUTPUT);
    pinMode(in_2, OUTPUT);
}

void Wheel::setSpeed(int speed) {
    int setSpeed = (int) (speed * _correction_factor); // Apply correction factor
    constrain(setSpeed, -255,255)
    if(setSpeed > 0) {
        forward(setSpeed);
    } else if (setSpeed < 0) {
        reverse(-setSpeed);
    } else {
        stop();
    }
}

void Wheel::forward(int speed) {
    if (_reverse) {
        digitalWrite(in_1, HIGH);
        digitalWrite(in_2, LOW);
    } else {
        digitalWrite(in_1, LOW);
        digitalWrite(in_2, HIGH);
    }
    analogWrite(pwm, speed);
}

void Wheel::reverse(int speed) {
    if (_reverse) {
        digitalWrite(in_1, LOW);
        digitalWrite(in_2, HIGH);
    } else {
        digitalWrite(in_1, HIGH);
        digitalWrite(in_2, LOW);
    }
    analogWrite(pwm, speed);
}

void Wheel::stop() {
    digitalWrite(in_1, LOW);
    digitalWrite(in_2, LOW);
    analogWrite(pwm, 0);
}