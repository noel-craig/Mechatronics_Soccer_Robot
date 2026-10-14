#include "Wheel.h"

Wheel::Wheel(int pwm_pin, int in1_pin, int in2_pin, bool dir_invert) {
    pwm = pwm_pin;
    in_1 = in1_pin;
    in_2 = in2_pin;
    reverse = dir_invert;
}

void Wheel::init() {
    pinMode(pwm, OUTPUT);
    pinMode(in_1, OUTPUT);
    pinMode(in_2, OUTPUT);
}

void Wheel::setSpeed(int speed) {
    if(speed > 0) {
        forward(speed);
    } else if (speed < 0) {
        reverse(-speed);
    } else {
        stop();
    }
}

void Wheel::forward(int speed) {
    if (reverse) {
        digitalWrite(in_1, HIGH);
        digitalWrite(in_2, LOW);
    } else {
        digitalWrite(in_1, LOW);
        digitalWrite(in_2, HIGH);
    }
    analogWrite(pwm, speed);
}

void Wheel::reverse(int speed) {
    if (reverse) {
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