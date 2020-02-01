/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Mathstronauts_ConditionalDelay.h
 * Author: Richard
 *
 * Created on January 22, 2017, 3:20 PM
 * Last modified on January 31, 2020 10:00 PM
 * Version 5.2
 */

#ifndef MATHSTRONAUTS_LIBRARY_H
#define MATHSTRONAUTS_LIBRARY_H
#endif

#ifndef SERVO_H
#define SERVO_H

#endif


#ifndef ON
#define ON  1
#endif
#ifndef OFF
#define OFF 0
#endif

#ifndef POS
#define POS  0
#ifndef NEG
#define NEG 1

#endif


      
#if ARDUINO >=100
  #include "Arduino.h"
  #include <Servo.h>
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

extern int RED;
extern int GREEN;
extern int BLUE;

void digitalMotor(int pin, int Action);
void conditionalDelay(int pin, int stopValue);
void nDeltaDelay(int pin, int nChanges);
void servoMode(int pin, int X);       
void servoWrite(int angle);
float ultrasonicDistance(int trig, int echo);
void hold(int pin);
void PWM(int pin, int dutyCycle);
void servoSweep(int angle);
int getMin(int* array, int size);
void colourDetection(int filterPin1, int filterPin2, int sensorOut);
int digitalJSRead(int pin);
int decrease(int var, int decrem);
int increase(int var, int increm);
void increaseServoAngle(Servo servo, int increm, int Max);
void decreaseServoAngle(Servo servo, int decrem, int Min);
void initializeServo(Servo servo,int pin, int initPos);
#endif /* MATHSTRONAUTS_CONDITIONALDELAY_H */