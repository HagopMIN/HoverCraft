
#ifndef HOVERCRAFT_H
#define HOVERCRAFT_H

#include "fans.c"
#include "Arduino.h"
#include <NoDelay.h>
#include <Servo.h>
#include "fans.h"
#include "NewPing.h" 


//SERVO
#define ServoPin 9
#define ServoDelay_ms 1
#define NinetyDeg 90

//Serial
#define BAUDRATE 9600

//US
#define F_TRIGGER_PIN 11
#define F_ECHO_PIN 2
#define L_TRIGGER_PIN 3
#define L_ECHO_PIN 13

#define SAMPLESIZE 50

// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400	


//STATE CONTROL
enum state {
  lift,
  moveForward,
  turnLeft,
  turnRight,
  idle,
  ex,
  s,
  dontn,
  sensortest,
  frontsensorcheck,
  sidesensorcheck,
  turn
};

//Function prototypes
void MPU_6050_Init(void);
void MPU_6050_ActiveCorrection(void);
#endif