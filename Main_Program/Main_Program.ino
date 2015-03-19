
#include <Servo.h> 
#include <Wire.h>
#include <LSM303.h>
#include "LIDAR.h"
 
//Course constants for searching
const int BaseWidth = 30; // cm, 1 foot
const int CourseWidth = 300; // cm, 3m
const int CourseLength = 300; //cm, 6m
const int SafteyFactor = 2;
const float North = 270.0;
const float NorthEast = 297.5;
const float South = 90.0;
const float East = 0;
const float West = 180.0;

const int trapDown = 155;
const int trapUp= 90;
 
//Speed
//Stop
const int LeftStop = 1378;
const int RightStop = 1336;
//Slow Forward
const int LeftSlowForward = 1310;
const int RightSlowForward = 1400;
//Medium Forward
const int LeftMediumForward = 1209;
const int RightMediumForward = 1500;
//Fast Forward
const int LeftFastForward = 1160;
const int RightFastForward = 1535;

//IR
const int LeftIRPipeValue = 160;
const int RightIRPipeValue = 140;

const int LeftIRFloorValue = 110;
const int RightIRFloorValue = 60;

const int LeftIRLimit = 110;
const int RightIRLimit = 50;

//Clamp constants
const int ClampUp = 180;
const int ClampDown = 0;
const int ClampBack = 180;
const int ClampCenter = 90;
const int ClampFront = 0;
 
const int ClampLeftHingeUp = 25;
const int ClampRightHingeUp = 155;

const int ClampLeftHingeDown = 155;
const int ClampRightHingeDown = 25;

const int ClampRightGripBack = 28;//20;//22;//17;
const int ClampLeftGripBack = 144;//151;//149;//155;

const int ClampRightGripForward = 144;//153;//155;
const int ClampLeftGripForward = 27;//22;
 
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

int servoArmPin = 6;
Servo servoArm;

//Clamp
int servoClampHingeLeftPin = 7;
Servo servoClampHingeLeft;
int servoClampHingeRightPin = 46;
Servo servoClampHingeRight;
int servoClampGripLeftPin = 45;
Servo servoClampGripLeft;
int servoClampGripRightPin = 44;
Servo servoClampGripRight;

int bumpTopPin = 19;
int bumpTopInterrupt = 4;
volatile boolean bumpTop = false; // Triggered true by boolean. 

int bumpBottomPin = 18;
int bumpBottomInterrupt = 5;
volatile boolean bumpBottom = false; // Triggered true by boolean. 

int IRLeftPin = A14;
int IRRightPin = A15;

Lidar lidar; //Create LIDAR object
int lidarMonitorPin = 16;
int lidarTriggerPin = 17;

void forward()
{
    servoLeft.write(60.5);  
    servoRight.write(97);
}

void forward(int leftSpeed, int rightSpeed)
{
    servoLeft.writeMicroseconds(leftSpeed);  
    servoRight.writeMicroseconds(rightSpeed);
}


void forward(int velPercent, float wantedHeading)
{

   // Normalize velocity{0..99}, heading{0..360}

   velPercent %= 100;

   while (wantedHeading < 0.0) wantedHeading += 360;

   while (wantedHeading >= 360) wantedHeading -= 360;

   

   compass.read();

   float _heading = compass.heading();

   //while (_heading < 0.0) _heading += 360;

   //while (_heading >= 360) _heading -= 360;

   

   float adjustAngle = wantedHeading - _heading;

   while (adjustAngle > 180) adjustAngle -= 360;

   while (adjustAngle < -180) adjustAngle += 360;

   

   float leftRange = LeftStop - LeftFastForward;

   float rightRange = RightFastForward - RightStop; 
   
   

   float leftSpeed = (velPercent*leftRange/100);

   float rightSpeed = (velPercent*rightRange/100);
   leftRange *= 2;
   rightRange *= 2;
  /* 
   Serial.println("-----");
   Serial.println(wantedHeading);
   Serial.println(_heading);
   Serial.println(adjustAngle);
   delay(200);
*/
   if (adjustAngle > 0) 

   {

      leftSpeed = leftSpeed + (leftRange - leftSpeed)*(adjustAngle)/180.0;

      rightSpeed = rightSpeed - rightSpeed*(adjustAngle)/180.0;

   }

   else

   {

      leftSpeed = leftSpeed - leftSpeed*(-adjustAngle)/180.0;

      rightSpeed = rightSpeed + (rightRange - rightSpeed)*(-adjustAngle)/180.0;

   }

   

   leftSpeed = LeftStop - leftSpeed;

   rightSpeed = RightStop + rightSpeed;

   

   servoLeft.writeMicroseconds((int)leftSpeed);

   servoRight.writeMicroseconds((int)rightSpeed);

}

void brake(){
    servoLeft.writeMicroseconds(1378);  
    servoRight.writeMicroseconds(1336);    
}

void reverse(){
    servoLeft.write(100.5);  
    servoRight.write(57);    
}

void left(){
    servoLeft.write(77.5);  
    servoRight.write(74);    
}

void right(){
    servoLeft.write(83.5);  
    servoRight.write(80);    
}

void turn(float offset)
{
  boolean temp = false;  
  compass.read();
  heading = compass.heading();
  float wantedHeading = initialHeading + offset;

  // This algorithm determines whether it is faster to turn right or left depending on the current heading.
  if(heading < initialHeading)
  {
    heading += 360;
  }
  
  if((heading - wantedHeading) < 180) // Modulus does not work on floats
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
  compass.read();
  boolean temp = false;  
  float tempheading = compass.heading();
  float tolerance = 2.0;
  float wantedHeading = initialHeading + offset;  

  if(wantedHeading > 360)
  {
    wantedHeading -= 360;
  }
  
  if((wantedHeading < tolerance) && (tempheading > (360-tolerance)))
  {
    wantedHeading += 360;
  }
  
  if((wantedHeading > (360 - tolerance)) && (tempheading < tolerance))
  {
    wantedHeading -= 360;
  } 
  
  float error = abs(wantedHeading - tempheading);
  // Compare with the initial vector.  
  
  if( error < tolerance )
  {
    temp = true;
  }
  
  return temp;
}

boolean onRamp(double percent) // Checks if on ramp.
{  
  float gravity = 0;
  for (int i = 0; i < 5; i ++)
  {
    compass.read();
    gravity += compass.a.z;
    delay(10);
  }
  
  gravity /= 5;
  //values are negative due to being mounted upside down, thus will be "bigger"
  if(initialGravity * percent < gravity)
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
  
  if( (leftIRValue < LeftIRLimit)&&(rightIRValue < RightIRLimit)) // Check left IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    forward(LeftSlowForward, RightSlowForward + 50);  
  }
  else if( leftIRValue < LeftIRLimit ) // Check left IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    right();
  }
  else if( rightIRValue < RightIRLimit ) // Check Right IR. Change to correct calibration cutoff.
  {
    // Add more complicated course adjustment.
    left();
  }
  else
  {
    forward(LeftSlowForward, RightSlowForward + 50);
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
  compass.read();
  float wantedHeading = compass.heading();
  while(!bumpTop || !bumpBottom || lidar.scan() < distance)
  {    
    forward(50, wantedHeading);
  }      
  brake();
}

//Returns time taken in us to get from one end of the base to the other
int calculateDelayTime(double distance)
{
  long time1 = micros();	
  compass.read();
  float wantedHeading = compass.heading();
  while(!bumpTop || !bumpBottom || lidar.scan() > distance)
  {    
    forward(50, wantedHeading);
  }    
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
    compass.read();
    float wantedHeading = compass.heading();
    while(!bumpBottom) 
    {    
      forward(50, wantedHeading);
    }
    brake();

    deployTrap();
    retractTrap();

    //Return to wall and orientate 180deg from search direction 
    turn(East + (90 * searchNumber) % 360);
    compass.read();
    wantedHeading = compass.heading();
    while(!bumpBottom || !bumpTop)
    {    
      forward(50, wantedHeading);
    }    
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
        compass.read();
        float wantedHeading = compass.heading();
        while(!bumpBottom || !bumpTop)
        {    
          forward(50, wantedHeading);
        }            
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
  compass.read();
  float wantedHeading = compass.heading();
  while(!(bumpTop))
  {
    forward(50, wantedHeading);
  }
}

void DeployClamp()
{
  for(int i = ClampUp; i > ClampDown; i-=5)
  {
    servoClampHingeLeft.write(i);
    servoClampHingeRight.write(i);
    delay (10);
  }
  servoClampGripLeft.write(ClampBack);
  servoClampGripRight.write(ClampBack);
}


void ClimbRamp()
{  
  //Test Grip
  /*
  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
  
  delay(1000);
 
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(ClampRightGripForward);
  
  while(!bumpTop);
  */
  forward(LeftFastForward, RightFastForward);

  //Segway Up Ramp
  for (int i = 0; i <= 105 ; i+=1)
  {
      servoClampGripLeft.write(ClampLeftGripForward + i);
      servoClampGripRight.write(ClampRightGripForward - i);
      delay(20);
  }

  delay(8000);
  
  //while(!onRamp(0.85))
  {
    //Transistion to driving up ramp
    delay(5000);
    servoArm.write(trapDown);
    delay(1000);
    servoClampGripLeft.write(ClampLeftGripBack);
    servoClampGripRight.write(160);

    delay(2000);
    servoClampGripRight.write(ClampRightGripBack);
    delay(2000);
  }

  delay(2000);
  
  //Traverse Top / Backwards segway
  //while(!onRamp(12700/15800));  
  while(onRamp(0.78));
    //forward(LeftSlowForward, RightSlowForward);
  servoClampGripLeft.write(ClampLeftGripBack + 15);
  servoClampGripRight.write(ClampRightGripBack - 15);
    
  //delay(200);
  //Reverse Drive
  //forward(RightFastForward, LeftFastForward);
  
  
  delay(750);
  
  
  // Forward down ramp
  forward(LeftSlowForward, RightSlowForward);
  servoArm.write(trapUp);
  
  //Get off when IR sense bottom
  /*while(analogRead(IRLeftPin) < LeftIRFloorValue && analogRead(IRRightPin) < RightIRFloorValue);  
  dely(100);
  servoClampHingeLeft.write(ClampLeftHingeUp);
  servoClampHingeRight.write(ClampRightHingeUp);
  servoClampGripLeft.write(90);
  servoClampGripRight.write(90);
  */
  while(onRamp(0.95));

}

void setup() 
{ 
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  servoArm.attach(servoArmPin);
  servoClampHingeLeft.attach(servoClampHingeLeftPin);
  servoClampHingeRight.attach(servoClampHingeRightPin);
  servoClampGripLeft.attach(servoClampGripLeftPin);
  servoClampGripRight.attach(servoClampGripRightPin);
  
  servoArm.write(trapUp);

  servoClampHingeLeft.write(ClampLeftHingeUp);
  servoClampHingeRight.write(ClampRightHingeUp);

  servoClampGripLeft.write(90);
  servoClampGripRight.write(90);
  
  brake();
  
  pinMode(IRLeftPin, INPUT);
  pinMode(IRRightPin, INPUT);
  
  pinMode(bumpTopPin, INPUT_PULLUP); 
  pinMode(bumpBottomPin, INPUT_PULLUP);
  
  //Attach bump ISRs
  attachInterrupt(bumpTopInterrupt, ISR_BUMP_TOP, CHANGE); // Note- can be LOW, CHANGE, RISING, or FALLING
  attachInterrupt(bumpBottomInterrupt, ISR_BUMP_BOTTOM, CHANGE); // Note- can be LOW, CHANGE, RISING, or FALLING
  
  while(!bumpTop);
    
  lidar.initPWM(lidarTriggerPin, lidarMonitorPin);
   
  Wire.begin();
  compass.init();
  compass.enableDefault();  
  //compass.m_min = (LSM303::vector<int16_t>){  -1341,   -2668,  +564};
  //compass.m_max = (LSM303::vector<int16_t>){  +1395,  +2177,  +3502};
  compass.m_min = (LSM303::vector<int16_t>){  -1179,  -2460,   -742};
  compass.m_max = (LSM303::vector<int16_t>){  +2222,  +3597,  +3278};
  
  delay(500);
  compass.read();  
  initialHeading = compass.heading();
  initialGravity = compass.a.z;
  onRampGravity = initialGravity*0.80; // This should be 0.707*z at 45 degrees;
  
  Serial.begin(9600);
  // Sets the servos to an initial position so that it does not move at start up 
} 


void forwardLidar(int leftSpeed, int rightSpeed)
{
 
  static int runningAvg; 
  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);  
  int reading = lidar.scan();
  delay(50);
  int reading2 = lidar.scan();
  // Error checking
  if (reading2 > 300 || reading > 300)
  {
    return;
  }
  if (runningAvg == 0)
  {
    //runningAvg = (reading + reading2) / 2;
  }
  else 
  {
    //runningAvg *= 0.75;
    //runningAvg += ((reading + reading2) / 2)*0.25;
  }
  // Cases
  int diff = reading2 - reading; 
  int diffTol = 10;
  
  if (abs(diff)  > diffTol)
  {
    // Assume we've seen a corner. Reset.
    //runningAvg = reading2;
    
    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);
  }
  else if (abs(diff) >= 1) // correction tolerance 
  {
    // Valid difference. Find direction
    //if (diff > 0) 
    {
      // Further away from wall. Correct towards Lidar
      servoLeft.write(leftSpeed + diff*10);
      servoRight.write(rightSpeed + diff*10);
    }
    /*else if (diff < 0)
    {
      servoLeft.write(leftSpeed-50);
      servoRight.write(rightSpeed-50);
    }*/
  }
  else
  {
    // Default if no error
    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);
  }
  
  //runningAvg *= 0.75; 
  //runningAvg
 delay(300); 
  
}

void GetOffBase()
{
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(155);
      delay(1000);

  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
  delay(1000);
  
  forwardLidar(1294, RightSlowForward);
  
  delay(5000);
  
  for (int i = 0; i < 20; i++)
  {
      servoClampGripLeft.write(ClampLeftGripForward - i);
      servoClampGripRight.write(155 + i);
      delay(60);
  }
  
  servoClampHingeLeft.write(ClampLeftHingeUp);
  servoClampHingeRight.write(ClampRightHingeUp);
  delay(1000);
  servoClampGripLeft.write(90);
  servoClampGripRight.write(90);
}

void loop() 
{ 
  
  //while(!bumpTop);

  GetOffBase();
  
  //Traverse to Pipe
  while(!bumpTop)
  {
    forwardLidar(1294, RightSlowForward);
  }
  
  forward(RightMediumForward, RightMediumForward);
  delay(1000);
  forward(LeftFastForward, RightMediumForward);

  while(!bumpTop && !onRamp(0.85));
  forward(LeftFastForward, RightFastForward);

  //Grip
  while(!onRamp(0.85));

    brake();
    delay(1000);
  servoClampGripLeft.write(ClampLeftGripForward +5);
  servoClampGripRight.write(ClampRightGripForward - 5);
    delay(1000);
  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
    delay(1000);
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(ClampRightGripForward);
    delay(1000);
   
  ClimbRamp();
  while(1)  

 forward(1294, RightSlowForward);

  lidar.on();
  int val = lidar.scan();
  while(val > 150)
  {
  
  val = lidar.scan();
  
  delay(20);
  }
  
  delay(3000);

  forward(RightMediumForward, RightMediumForward);
  delay(4000);
  forward(1294, RightSlowForward);
  while(!bumpTop)
  {
    forwardLidar(1294, RightSlowForward);
  }
  brake();
  
  servoArm.write(trapDown);
  delay(2000);
  servoArm.write(trapUp);
  delay(500);
  
  forwardLidar(RightSlowForward, 1294);
  
  delay(2000);
  
  forward(RightMediumForward, RightMediumForward);
  delay(9900); 
  
  forward(1294, RightSlowForward);
  delay(20000);
  forward(LeftMediumForward, LeftMediumForward);
  delay(3300);
  forward(LeftMediumForward, RightFastForward);

  
   
  while(1);
/*
  /// Forward unitl hitting bump sensor.
  //forward(LeftFastForward, RightFastForward);
  //while(1);
  //delay(20000);
  //turn(NorthEast);
   // while(1);

  //servoClampGripLeft.write(ClampLeftGripBack + 10);
  //servoClampGripRight.write(ClampRightGripBack - 10);
  //Due to bump sensor not being complete use IR for now
  compass.read();
  float head = compass.heading();
  //long time = millis();
  //while(analogRead(IRLeftPin) < LeftIRPipeValue && analogRead(IRRightPin) < RightIRPipeValue)
  
  while(1)
  {
    forward(50, head);
  }
  
  brake();
  while(1);
  //turn(NorthEast);
  while(!onRamp(0.85))
  {
     forward(50, head - 90);
  }

  //while(!bumpTop);
  
  // Turn until the right heading
  //turn(NorthEast);
  delay(2000);
  int left = LeftFastForward;
  int right = RightFastForward;
 // forward(left, right);
  
  compass.read();
  float head = compass.heading();
  */
  forward(LeftFastForward, RightFastForward);
  
  // Add course corrections as needed.
  
  while(!onRamp(0.75));
  brake();
    delay(1000);
  servoClampGripLeft.write(ClampLeftGripForward +5);
  servoClampGripRight.write(ClampRightGripForward - 5);
    delay(1000);
  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
    delay(1000);
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(ClampRightGripForward);
    delay(1000);
  
    
  ClimbRamp();

  brake(); 
 
  // Search find and capture lego figure
  gridSearch();
  
  // Add course corrections as needed.
  // Drive forward until on ramp.
  forward(LeftMediumForward, RightFastForward);
  ClimbRamp();

  brake(); 
  
  // Dead reckoning back to base
  forward(LeftFastForward, RightFastForward);
  while(!bumpTop);
  delay(1000); //adjust so that it travels the right distance.
  turn(West);  // Line up with base.  
  moveUntilBump();
  deployTrap(); // Safely deposit person at base 2.  
  
  while(1) // do nothing after the main loop.
  {
  }
}

void ISR_BUMP_TOP()
{
  // Double check that arduinon automaticaly resets the register for me.
  
  //Pin is normally High, Low when button pressed
  Serial.println("TOP");
  Serial.println(digitalRead(bumpTopPin));
  bumpTop = (digitalRead(bumpTopPin) == LOW);
  Serial.println(bumpTop);
}

void ISR_BUMP_BOTTOM()
{
  // Double check that arduinon automaticaly resets the register for me.
  
  //Pin is normally High, Low when button pressed
  Serial.println("bottom");
  bumpBottom = (digitalRead(bumpBottomPin) == LOW);
}
