#ifndef FANS_H
#define FANS_H

#include "Arduino.h"


#define Throttle_Pin 6 //P3 
#define Lift_Pin 5 //P4


#define T_MaxSpeed 100
#define T_MinSpeed 255 

#define T_SixtySpeed 150
#define T_STOP 0
#define L_MaxSpeed 255
#define L_MinSpeed 255

//Function Prototypes
void initialize_throttle_fan(); 
void initialize_lift_fan();
void set_throttle_speed(int speed);
void set_lift_speed(int speed);
int get_throttle_speed(void);
int get_lift_speed(void);
#endif