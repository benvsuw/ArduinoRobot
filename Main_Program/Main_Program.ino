#include <Servo.h> 
#include <Wire.h>
#include "LSM303.h"
#include "LIDAR.h"
 
//Course constants for searching
const int BaseWidth = 30; // cm, 1 foot
const int CourseWidth = 300; // cm, 3m
const int CourseLength = 600; //cm, 6m
const int SafteyFactor = 2;
const float North = 90.0;
const float South = 270.0;
const float East = 0;
const float West = 180.0;
 
LSM303 compass;
float heading;
float initialHeading;
float gravity;
float initialGravity;
float onRampGravity;
 
int servoLeftPin = 4;
int servoRightPin = 5;
Servo servoLeft;  // create servo object to control a servo
Servo servoRight;  // create servo object to control a servo

int servoArmPin = 7;
Servo servoArm;

int servoFlangePin = 6;
Servo servoFlange;

int bumpTopPin = 18;
int bumpTopInterrupt = 4;
volatile boolean bumpTop = false; // Triggered true by boolean. 

int bumpBottomPin = 19;
int bumpBottomInterrupt = 5;
volatile boolean bumpBottom = false; // Triggered true by boolean. 

int IRLeftPin = 0;
int IRRightPin = 1;

Lidar lidar; //Create LIDAR object
int lidarMonitorPin = 16;
int lidarTriggerPin = 17;

void forward(){
    servoLeft.write(0);  
    servoRight.write(180);    
}

void brake(){
    servoLeft.write(85);  
    servoRight.write(85);    
}

void reverse(){
    servoLeft.write(180);  
    servoRight.write(0);    
}

void left(){
    servoLeft.write(0);  
    servoRight.write(0);    
}

void right(){
    servoLeft.write(180);  
    servoRight.write(180);    
}

void turn(float offset)
{
  boolean temp = false;  
  heading = compass.heading();
  float wantedHeading = initialHeading + offset;
  
  // This algorithm determines whether it is faster to turn right or left depending on the current heading.
  if(heading < initialHeading)
  {
    heading += 360;
  }
  if((heading - wantedHeading) > 180) // Modulus does not work on floats
  {
    right();
  }
  else
  {
    left();
  }  
  while(!headingCheck(offset)){};  // Line up with base.
}

boolean headingCheck(double offset)  // Checks that the heading is correct.
{
  boolean temp = false;  
  heading = compass.heading();
  float wantedHeading = initialHeading + offset;
  if(wantedHeading > 360) // Modulus does not work on floats
  {
    wantedHeading -= 360;
  }
  // Compare with the initial vector.  
  if( (initialHeading <= (wantedHeading + 0.5))&&(initialHeading >= (wantedHeading - 0.5)) )
  {
    temp = true;
  }
  return temp;
}

boolean onRamp() // Checks if on ramp.
{  
  compass.read();
  if(onRampGravity > compass.a.z)
  {
    return true;
  }  
  return false;
}

void stayOnRamp()
{
  // Check two IR Sensors  
  int leftIRValue = analogRead(IRLeftPin);   
  int rightIRValue = analogRead(IRRightPin); 
  
  if( (leftIRValue < 80)&&(rightIRValue < 80)) // Check left IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    forward();
  }
  if( leftIRValue < 80 ) // Check left IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    right();
  }
  else if( rightIRValue < 80 ) // Check Right IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    left();
  }
  else
  {
    forward();
  }
}

void deployTrap()
{
  //TODO:: Servo control Code
}

void retractTrap()
{
  //TODO:: Servo Control Code
}


//Searches for an item that should be x cm away, stop once found or if a boundary hit
void search(int distance)
{
  forward();
  while(!bumpTop || !bumpBottom || lidar.scan() < distance);
  brake();
}

//Returns time taken in us to get from one end of the base to the other
int calculateDelayTime(double distance)
{
  long time1 = micros();	
  forward();
  while(!bumpTop || !bumpBottom || lidar.scan() > distance);	
  brake();	
  return (int)((micros() - time1) / 2);
}

//Determines search result (found base, hit base, hit wall/pipe)
//Executes appropriate response
bool parseGridSearchResult(int searchNumber, double distance)
{
  if(bumpTop)
  {
    return false;
  }
  else if(bumpBottom) 
  {
    //Case needs to be beefed up so that the robot centers itself on base
    deployTrap();
    retractTrap();
    turn(South + (90 * searchNumber) % 360);
    return true;
  }
  else if(lidar.scan() < distance)
  {
    //Find time taken to get to other end of base
    int delayTime = calculateDelayTime(distance);
    
    //Reverse till centered            
    reverse();
    delayMicroseconds(delayTime);
    brake();
    
    //Turn to be perpindicular to base and go forward till the base is hit
    turn(West + (90 * searchNumber) % 360);
    forward();
    while(!bumpBottom);
    brake();

    deployTrap();
    retractTrap();

    //Return to wall and orientate 180deg from search direction 
    turn(East + (90 * searchNumber) % 360);
    forward();
    while(!bumpBottom || !bumpTop);
    brake();
    turn(South + (90 * searchNumber) % 360);
    return true;
  }
  return false;
}

//Function for finding and capturing lego man
//Final posistion wil be against the east wall facing south
void gridSearch()
{
  int maxWidth = CourseWidth - BaseWidth / SafteyFactor;
  int maxLength = CourseLength - BaseWidth / SafteyFactor;

  lidar.on();
  //Shouldn't need to run all four walls, but code is here in case it is necessary
  for(int i = 0; i < 4; i++)
  {
    double distance = 0;
    if(i % 2 == 0)
      distance = maxWidth;
    else 
      distance = maxLength;
		
    search(distance);		
    
    if(parseGridSearchResult(i, distance))
    {
      //Traverse to be back on the east wall facing south
      for (int j = 0; j < i; j++)
      {
        forward();
        while(!bumpBottom || !bumpTop);
        brake();
        turn(South + (90 * j) % 360);
      }
      break;
    }
    else //Check next wall
    {
      turn(West + (90 * i) % 360);
    }
  }
  lidar.off();
}

void moveUntilBump()
{
  forward();
  while(!(bumpTop)){}
}

void setup() 
{ 
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  
  pinMode(IRLeftPin, INPUT);
  pinMode(IRRightPin, INPUT);
  
  pinMode(bumpTopPin, INPUT_PULLUP); 
  pinMode(bumpBottomPin, INPUT_PULLUP);
  
  //Attach bump ISRs
  attachInterrupt(bumpTopInterrupt, ISR_BUMP_TOP, CHANGE); // Note- can be LOW, CHANGE, RISING, or FALLING
  attachInterrupt(bumpBottomInterrupt, ISR_BUMP_BOTTOM, CHANGE); // Note- can be LOW, CHANGE, RISING, or FALLING
  
  lidar.initPWM(lidarTriggerPin, lidarMonitorPin);
  
  Wire.begin();
  compass.init();
  compass.enableDefault();  
  compass.m_min = (LSM303::vector<int16_t>){  -1148,   -681,  -3836};
  compass.m_max = (LSM303::vector<int16_t>){  +1418,  +1286,  -1843};
  
  compass.read();  
  initialHeading = compass.heading();
  initialGravity = compass.a.z;
  onRampGravity = initialGravity*0.72; // This should be 0.707*z at 45 degrees;
  
  delay(500);  
} 

void loop() 
{
  // Forward unitl hitting bump sensor.
  moveUntilBump();
  
  // Turn until the right heading
  turn(90.0);
  
  // Add course corrections as needed.
  
  // Drive forward until on ramp.
  forward();  
  while(!(onRamp()))
  {
  delay(10);
  } 
  
  // Initial up ramp
  while(onRamp())
  {
    stayOnRamp();
    delay(10);
  }
  
  // At the corner it drives straight over.
  forward();
  while(!onRamp())
  {
  delay(10);
  } 
  
  // Going down ramp
  while(onRamp())
  {
    stayOnRamp();
  }  
  brake(); 
  
  // Search find and capture lego figure
  gridSearch();
  
  // Add course corrections as needed.
  // Drive forward until on ramp.
  forward();  
  while(!(onRamp())){}; // Set condition to check if on ramp.
  
  // Initial up ramp
  while(onRamp())
  {
    stayOnRamp();
  }
  
  // At the corner it drives straight over.
  forward();
  while(!onRamp()){}
  
  // Going down ramp
  while(onRamp())
  {
    stayOnRamp();
  }  
  brake(); 
  
  // Dead reckoning back to base
  forward();
  delay(1000); //adjust so that it travels the right distance.
  turn(180);  // Line up with base.  
  moveUntilBump();
  servoArm.write(0); // Safely deposit person at base 2.  
  
  while(1) // do nothing after the main loop.
  {
  }
}

void ISR_BUMP_TOP()
{
  // Double check that arduinon automaticaly resets the register for me.
  
  //Pin is normally High, Low when button pressed
  bumpTop = (digitalRead(bumpTopPin) == LOW);
}

void ISR_BUMP_BOTTOM()
{
  // Double check that arduinon automaticaly resets the register for me.
  
  //Pin is normally High, Low when button pressed
  bumpBottom = (digitalRead(bumpBottomPin) == LOW);
}
