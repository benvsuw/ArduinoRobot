/*
  Ultra.h - Library for communicating with HC-SR04
  Created by Benjamin G. Eckert, January 15, 2015.
*/

#ifndef Ultra_h
#define Ultra_h

#include<stdlib.h>
#include "Arduino.h"

class Ultra
{
  public:
    Ultra(int pinTrig, int pinEcho);
    double scan(); // Outputs a double representing the distance in millimetres.
  private:
	int _pinTrig;
	int _pinEcho;
	long pulse(); // returns a single reading of an echo
	double convertToCm( long time );
};

#endif

