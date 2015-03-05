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
    Lidar(int pinTrig, int pinEcho);
    int scan(); // Return distance in mm. 
    
    
  private:
  bool _init;
  int _pinTrig;
  int _pinEcho;
  
  int scanPWM();
  void initPWM();
  int scanI2C();
  void initI2C();
  int scanWire();
  void initWire();
};

#endif

