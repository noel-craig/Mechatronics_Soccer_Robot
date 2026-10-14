#include <Arduino.h>
#include "Chassis.h"
#include "Wheel.h"
#include "PinAssignments.h"

// HARDWARE PINS
const int right_signal_pin = A0;
const int left_signal_pin = A1;

Wheel wheels[] = {
  Wheel(FL_PWM, FL_DIR1, FL_DIR2, false), //fl
  Wheel(FR_PWM, FR_DIR1, FR_DIR2, true), //fr
  Wheel(BL_PWM, BL_DIR1, BL_DIR2, false), //bl
  Wheel(BR_PWM, BR_DIR1, BR_DIR2, true) //br
};

Chassis chassis = Chassis(
  wheels[0], // front left
  wheels[1], // front right
  wheels[2], // back left
  wheels[3]  // back right
);

// variables updated in loop
int right_intensity = 0;
int left_intensity = 0;
int speed_gain = 0;
int motor_speed = 0;
int motorspeeddiff = 0;
int leftmotorspeed = 0;

int rightmotorspeed = 0;


int thresh = 200; // off the white tape

// void spin(char *direction, int motor_speed){
//   if (strcmp(direction, "CCW") == 0){
//     Serial.println("spinning CCW");
//     for (int i = 0; i < num_wheels; i+=2){
//       int j = i + 1;
//       wheelForward(i, motor_speed);
//       wheelReverse(j, motor_speed);
//     }
//   }
//   else if (strcmp(direction, "CWO") ==0){
//     Serial.println("spinning CWO");
//     for (int i = 0; i < num_wheels; i+=2){
//       int j = i + 1;
//       wheelForward(j, motor_speed);
//       wheelReverse(i, motor_speed);
//     }
//   }
//   else{
//     Serial.println("Wrong direction input for spin operation. Stopping.");
//     stop();
//   }
// }

// void strafe(char *direction, int motor_speed){
//   if (strcmp(direction, "right") ==0){
//     // front_left + rear_right forward
//     for (int i = 1; i < 2; i++){
//         wheelForward(i, motor_speed);
//     }
//     // front_right + rear_left reverse
//     for (int j = 0; j < num_wheels; j +=3){
//         wheelReverse(j, motor_speed);
//     } 
//   }
//   else if (strcmp(direction, "left") ==0){
//     // front_left + rear_right reverse
//     for (int i = 1; i < 2; i++){
//       wheelReverse(i, motor_speed);
//     }
//     // front_right + rear_left reverse
//     for (int j = 0; j < num_wheels; j +=3){
//         wheelForward(j, motor_speed);
//     } 
//   }
//   else{
//     Serial.println("Wrong_direction input for strafe operation. Stopping.");
//     stop();
//   }
// }

// void Terry(int motor_speed){
//   for (int i = 0; i < num_wheels; i++){
//     wheelReverse(i, motor_speed);
//   }
// }

// void stop(){
//   for (int i = 0; i < num_wheels; i++){
//     wheelForward(i, 0);
//   } 
// }

// void initialize(){
//   spin("CWO", 255); delay(1000);
//   spin("CCW", 255); delay(1000);
//   stop();delay(1000);
// }

void setup() {

  Serial.begin(11500);
  chassis.init();
}

void loop() {

}