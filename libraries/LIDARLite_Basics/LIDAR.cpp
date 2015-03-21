/*
  LIDAR.h - Library for communicating with LIDARLite
  Created by Benjamin Vander Schaaf, MArch 4, 2015.
*/

#include<stdlib.h>
#include "Arduino.h"
#include "LIDAR.h"
#include <Wire.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

 /************************************************************************
 *  Public Functions                          		                     *
 ************************************************************************/
 
 // Constructor
 Lidar::Lidar(void) { }

 void Lidar::initPWM(int pinTrig, int pinEcho)
 {
 		_pinEcho = pinEcho;
 		_pinTrig = pinTrig;
 	  pinMode(_pinTrig, OUTPUT); // Set pin 2 as trigger pin
      pinMode(_pinEcho, INPUT); // Set pin 3 as monitor pin
      digitalWrite(_pinTrig, HIGH); // Set LIDAR off
      _on = false;
      _init = true;
 }
 
 void Lidar::on()
 {
 	if(!_on)
	{
 		digitalWrite(_pinTrig, LOW);
 		_on = true;
 	}
 }
 
 void Lidar::off()
 {
 	if(_on)
 	{
 		digitalWrite(_pinTrig, HIGH);
 		_on = false;
 	}
 }
 
 // Get distance in cm
 int Lidar::scan()
 {
    if(!_on)
    	on();
    
    long val = scanPWM();
    
    while(val == 0)
    {
    	off();
    	delay(200);
    	on();
    	delay(200);
    	val = scanPWM();
    }
    
    return val;
 }
 
 
  /************************************************************************
 *  Private Functions   	                                             *
 ************************************************************************/
  int Lidar::scanPWM()
  {   
    unsigned long pulse_width = 0;
    
	pulse_width = pulseIn(_pinEcho, HIGH);
    return pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
  }
  

  
  /*
  int Lidar::scanI2C()
  {
    if (!_init)
    {
      initI2C();
    }
    
      // Write 0x04 to register 0x00
    uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
    while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
      nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
      delay(1); // Wait 1 ms to prevent overpolling
    }
  
    byte distanceArray[2]; // array to store distance bytes from read function
    
    // Read 2byte distance from register 0x8f
    nackack = 100; // Setup variable to hold ACK/NACK resopnses     
    while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
      nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
      delay(1); // Wait 1 ms to prevent overpolling
    }
    int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
    
    // Print Distance
    // Serial.println(distance);
    return distance;
  }
  
  void Lidar::initI2C()
  {
    //Serial.begin(9600); //Opens serial connection at 9600bps.     
    I2c.begin(); // Opens & joins the irc bus as master
    delay(100); // Waits to make sure everything is powered up before sending or receiving data  
    I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
    
    _init = true;
  }
  
  int Lidar::scanWire()
  {

    int reading = 0;
    Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
    Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
    Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
    Wire.endTransmission(); // stop transmitting
  
    delay(20); // Wait 20ms for transmit
  
    Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
    Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
    Wire.endTransmission(); // stop transmitting
  
    delay(20); // Wait 20ms for transmit
  
    Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
  
    if(2 <= Wire.available()) // if two bytes were received
    {
      reading = Wire.read(); // receive high byte (overwrites previous reading)
      reading = reading << 8; // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits
      // Serial.println(reading); // print the reading
      return reading;
    }
    
    return -1; // Error reading
  }
  
  void Lidar::initWire()
  {
    Wire.begin(); // join i2c bus
    // Serial.begin(9600); // start serial communication at 9600bps
   
    _init = true;
  }
  */
 
