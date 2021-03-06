/*
  LIDAR.h - Library for communicating with LIDARLite 
  Created by Benjamin Vander Schaaf, March 4, 2015.
*/

#ifndef LIDAR_h
#define LIDAR_h

#include<stdlib.h>
#include "Arduino.h"

class Lidar
{
  public:
    Lidar(void);
    void initPWM(int pinTrig, int pinEcho);		// Initialize LIDAR as PWM input
    int scan(); 								// Return distance in cm. 
    void on(); 									// Turn on LIDAR
    void off(); 								// Turn off LIDAR
    
    
  private:
  bool _init;
  bool _on;
  int _pinTrig;
  int _pinEcho;
  int scanPWM();
  /*
  void initPWM();
  int scanI2C();
  void initI2C();
  int scanWire();
  void initWire();
  */
};

#endif

