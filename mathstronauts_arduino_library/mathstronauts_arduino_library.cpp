/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Richard
 *
 * Created on December 6, 2016, 1:22 PM
 * Last modified on January 31, 2020, 10:00 PM
 * Version 5.2
 */

#include "Arduino.h"
#include "Servo.h"
#include "mathstronauts_arduino_library.h"
/*
 * To use as conditional delay, functions exists the desired number of changes occurred at the input digital pin
 * No operations occur while function is running
 *  */
 
int RED;
int GREEN;
int BLUE;

Servo servoMain; 
void conditionalDelay(int pin, int stopValue){
	int pinState = digitalRead(pin);
	while (pinState != stopValue){
      pinState = digitalRead(pin);
    }
}

void nDeltaDelay(int pin, int nChanges){
    int pinState;
    pinState = digitalRead(pin);
    if (nChanges ==1){
        while (digitalRead(pin)==pinState){
            Serial.println(digitalRead(pin));

        }
      }
    
    if(nChanges ==2){
    
        while (digitalRead(pin)==pinState){
            Serial.println(digitalRead(pin));
        }
        while(digitalRead(pin)== !pinState){
            Serial.println(digitalRead(pin));
        }
    }
    
    
    if(nChanges ==3){
        while (digitalRead(pin)==pinState){
            Serial.println(digitalRead(pin));
        }
        while(digitalRead(pin)== !pinState){
            Serial.println(digitalRead(pin));
        }
        while(digitalRead(pin)== pinState){
            Serial.println(digitalRead(pin));
        }
    }
}
/*
 Used to initalize pin that servo is attaced
 */
void servoMode(int pin, int X){
    //Servo servoMain;
    servoMain.attach(pin);
}
/* 
 Used to change angle of servo
 */
void servoWrite(int angle){
	//Servo servoMain;
    servoMain.write(angle);
}

void digitalMotor(int pin, int Action){
    if(pin==12){
        pinMode(9, OUTPUT);
        if(Action==1){
            digitalWrite(12, HIGH);
            digitalWrite(9,LOW);
            analogWrite(3, 255);
        }
        else{
            digitalWrite(9,HIGH);
        }
    }
    else if(pin==13){
        pinMode(8, OUTPUT);
        if(Action==1){
            digitalWrite(13, HIGH);
            digitalWrite(8,LOW);
            analogWrite(11, 255);
        }
        else{
            digitalWrite(8,HIGH);
        }
    }
    else{
        
    }
	}
	
float ultrasonicDistance(int trig, int echo){
	float duration;
	float distance;
	digitalWrite(trig, LOW);
	delayMicroseconds(2000);
	digitalWrite(trig, HIGH);
	delayMicroseconds(15);
	digitalWrite(trig, LOW);
	delayMicroseconds(10);
	duration = pulseIn(echo, HIGH);
	duration = duration/1000000.0;
	distance = duration*34300.0/2.0;
	if (distance < 0) {
	  distance = ultrasonicDistance(trig, echo);
	}
	return distance;
}

void hold(int pin){
	int state = digitalRead(pin);
	int initial = state;
	while (state == initial) {
        state = digitalRead(pin);
    }
}

void PWM(int pin, int dutyCycle){
	float factor = (float)(dutyCycle/(float)100);
	int signal=(int)(255*factor);
	analogWrite(pin,255);
	delay(100);
	analogWrite(pin, signal);
}

void servoSweep(int angle){
	for (int pos = 0; pos <= angle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoWrite(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

int getMin(int* array, int size)
  {
    int minimum = array[0];
    for (int i = 0; i < size; i++)
    {
      if (array[i] < minimum) minimum = array[i];
    }
    return minimum;
  }
  
  void colourDetection(int filterPin1, int filterPin2, int sensorOut){
	 RED =0;
	 GREEN =0;
	 BLUE=0;
	 int  s[3];
    // Setting red filtered photodiodes to be read
    int Minimum;
    digitalWrite(filterPin1,LOW);
    digitalWrite(filterPin2,LOW);
    // Reading the output  uency
     s[0]= pulseIn(sensorOut, LOW);
  
    //delay(10);
    // Setting Green filtered photodiodes to be read
    digitalWrite(filterPin1,HIGH);
    digitalWrite(filterPin2,HIGH);
    // Reading the output  uency
    //green  = pulseIn(sensorOut, LOW);
     s[1]= pulseIn(sensorOut, LOW);
    //delay(10);
    
    // Setting Blue filtered photodiodes to be read
    digitalWrite(filterPin1,LOW);
    digitalWrite(filterPin2,HIGH);
    // Reading the output  uency
    //blue  = pulseIn(sensorOut, LOW);
     s[2]= pulseIn(sensorOut, LOW);
    delay(10);
    Minimum = getMin( s,3);
    if (Minimum>100){
      RED  = 0;
      GREEN  =0;
      BLUE  =0;
    }
    else if(Minimum== s[0]){
      RED  = 255;
      GREEN =0;
      BLUE =0;
    }
    else if(Minimum== s[1]){
      RED  = 0;
      GREEN =255;
      BLUE =0;
    }
  
    else{
      RED  = 0;
      GREEN =0;
      BLUE =255;
    }
	
	  /* // Printing the value on the serial monitor
	  Serial.print("R= ");//printing name
	  Serial.print(RED);//printing RED color  uency
	  Serial.print("  ");
	  
	  // Printing the value on the serial monitor
	  Serial.print("G= ");//printing name
	  Serial.print(GREEN);//printing RED color  uency
	  Serial.print("  ");
	  
	  // Printing the value on the serial monitor
	  Serial.print("B= ");//printing name
	  Serial.println(BLUE);//printing RED color  uency
	  //Serial.print("  "); */
  }
  
  int digitalJSRead(int pin){
	int pin_State = analogRead(pin);
	if(pin_State >1000){
		return 1;
	}
	else if (pin_State <100){
		return 2;
	}
	else{
		return 0;
  }
}

int decrease(int var, int decrem){
  var = var - decrem;
  return var;
}

int increase(int var, int increm){
  var = var + increm;
  return var;
}


void increaseServoAngle(Servo servo, int increm, int Max){
  if(servo.read()- increm < Max){
    servo.write(servo.read() + increm); 
  } 
}

void decreaseServoAngle(Servo servo, int decrem, int Min){
  if(servo.read()- decrem > Min){
    servo.write(servo.read() - decrem); 
  } 
}

void initializeServo(Servo servo,int pin, int initPos){
  servo.attach(pin);
  servo.write(initPos);
}
  
	  
  


