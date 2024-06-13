#include "fans.h"
#include "Arduino.h"


//Throttle
const int T_maxSpeed = 255; // Maximum speed for PWM (0-255)
const int T_minSpeed = 40;
const int T_increment = 1;  // Speed increment
const int T_delayTime = 100; // Delay time in milliseconds
int T_fanspeed; 

//Lift 
const int L_maxSpeed = 255; // Maximum speed for PWM (0-255)
const int L_minSpeed = 40;
const int L_increment = 1;  // Speed increment
const int L_delayTime = 100; // Delay time in milliseconds
int L_fanspeed; 

void initialize_throttle_fan(void){
  pinMode(Throttle_Pin, OUTPUT);
  T_fanspeed = 0;
}

void initialize_lift_fan(void){
  pinMode(Lift_Pin, OUTPUT);
  L_fanspeed = 0; 
}

void set_throttle_speed(int speed){
  analogWrite(Throttle_Pin, speed);
}

void set_lift_speed(int speed){
  analogWrite(Lift_Pin, speed);
}


int get_throttle_speed(void){
  return T_fanspeed; 
}

int get_lift_speed(void){
  return L_fanspeed;
}
