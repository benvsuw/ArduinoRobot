/*
  Ultra.h - Library for communicating with HC-SR04
  Created by Benjamin G. Eckert, January 15, 2015.
*/

#include<stdlib.h>
#include "Arduino.h"
#include "Ultra.h"

 /************************************************************************
 *  Public Functions                          		                     *
 ************************************************************************/

// Constructor
Ultra::Ultra(int pinTrig, int pinEcho)
{
// Set direction registers
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
// Place parameters into local variables
  _pinTrig = pinTrig;
  _pinEcho = pinEcho;
}

// Outputs a double representing the distance in millimetres.
double Ultra::scan()
{	
	double output;
	output = convertToCm(pulse());
	return (output);
}

 /************************************************************************
 *  Private Functions   	                                             *
 ************************************************************************/

long Ultra::pulse() // Returns the time of the echo of the pulse in microseconds.
{
	long temp;
	pinMode(_pinTrig, OUTPUT);
	digitalWrite(_pinTrig, LOW);
	delayMicroseconds(2);
	digitalWrite(_pinTrig, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pinTrig, LOW);	
	pinMode (_pinEcho, INPUT);//attach pin 4 to Echo
	temp = pulseIn(_pinEcho, HIGH);	
	return temp;
}

double Ultra::convertToCm( long time )
{
	// Speed of sound in air is 340.29 m/s
	
	double distance;
	distance = time / 58.0;
	return (distance);
}

