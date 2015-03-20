
#include <Servo.h> 
#include <Wire.h>
#include <LSM303.h>
#include "LIDAR.h"

const int trapDown = 155;
const int trapUp= 90;
 
//Speed
//Stop
const int LeftStop = 1378;
const int RightStop = 1336;
//Slow Forward
const int LeftSlowForward = 1294;
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

const int LeftIRFloorValue = 120;
const int RightIRFloorValue = 90;

const int LeftIRLimit = 130;
const int RightIRLimit = 50;

//Clamp constants
const int ClampLeftHingeUp = 25;
const int ClampRightHingeUp = 155;

const int ClampLeftHingeDown = 155;
const int ClampRightHingeDown = 25;

const int ClampRightGripBack = 27;//20;//22;//17;
const int ClampLeftGripBack = 145;//151;//149;//155;

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

void brake(){
    servoLeft.writeMicroseconds(1378);  
    servoRight.writeMicroseconds(1336);    
}

void reverse(){
    servoLeft.write(100.5);  
    servoRight.write(57);    
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

void ClimbRamp()
{  
  forward(LeftFastForward, RightFastForward);

  //Segway Up Ramp
  for (int i = 0; i <= 97 ; i+=1)
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
    delay(1500);
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
  
  delay(750);
  
  
  // Forward down ramp
  forward(LeftSlowForward, RightSlowForward);
  servoArm.write(trapUp);
  
  //Get off when IR sense bottom
  while(analogRead(IRLeftPin) < LeftIRFloorValue && analogRead(IRRightPin) < RightIRFloorValue);  
  delay(8000);

  
  while(onRamp(0.96));
  delay(5000);
  servoClampHingeLeft.write(ClampLeftHingeUp);
  servoClampHingeRight.write(ClampRightHingeUp);
  delay(1000);
  servoClampGripLeft.write(90);
  servoClampGripRight.write(90);
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
    
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
  }
  else if (abs(diff) >= 1) // correction tolerance 
  {
    // Valid difference. Find direction
    //if (diff > 0) 
    {
      // Further away from wall. Correct towards Lidar
      servoLeft.writeMicroseconds(leftSpeed + diff*10);
      servoRight.writeMicroseconds(rightSpeed + diff*10);
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
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
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
  
  forward(LeftSlowForward, RightSlowForward);
  
  delay(10000);
  
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

void TurnLeftWall()
{
      // Initialize position
    servoClampHingeLeft.write(25); // 25
    servoClampHingeRight.write(155); //155
    delay(1000);
  
    servoClampGripLeft.write(144); //144
    servoClampGripRight.write(155); //155
    delay(1000);
    
// First rotation
    {
      servoClampHingeLeft.write(25+54); // 25+54
      servoClampHingeRight.write(155-54); //155-54
      delay(1000);
      
      servoClampGripLeft.write(27); //27
      servoClampGripRight.write(28); //28
      delay(1000);
      
      servoClampHingeLeft.write(25);//25
      servoClampHingeRight.write(155);//155
      delay(1000);
      
      servoClampGripLeft.write(144+20);//144+20
      servoClampGripRight.write(144+20);//144+20
      delay(1000);
    }
// Second rotation
    {
      servoClampHingeLeft.write(25+48);//25+48
      servoClampHingeRight.write(155-48);//155-48
      delay(1000);
      
      servoClampGripLeft.write(27);//27
      servoClampGripRight.write(28);//28
      delay(1000);
      
      servoClampHingeLeft.write(25);//25
      servoClampHingeRight.write(155);//155
      delay(1000);
      
      servoClampGripLeft.write(144);//144
      servoClampGripRight.write(144);//144
      delay(1000);
    }
    
    servoClampHingeLeft.write(ClampLeftHingeUp);
    servoClampHingeRight.write(ClampRightHingeUp);
    delay(1000);
    servoClampGripLeft.write(90);
    servoClampGripRight.write(90);
    delay(1000);
}


void loop() 
{ 
  //while(!bumpTop);
 
  GetOffBase();
  
  //Traverse to Pipe
  forward(LeftSlowForward, RightSlowForward);
  while(!bumpTop);
  
  forward(RightMediumForward, LeftMediumForward);
  delay(800);
  
  forward(RightMediumForward, RightMediumForward);
  delay(355);
  
  // 4 wheel drive
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(155);
  delay(1000);

  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
  delay(1000);
  
  forward(LeftFastForward, RightFastForward);
  //Grip ramp
  while(!onRamp(0.6));
  
  brake();
  
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
  delay(4000);
  forward(1294, RightSlowForward);
  
  lidar.on();
  delay(200);
  int val = lidar.scan();
  
  while(val > 150)
  {
    delay(50);
    val = lidar.scan();
  }
  lidar.off();

  delay(3000);

  //forward(RightMediumForward, RightMediumForward);
  //delay(1250);
  
  //Turn Left
  brake();
  
  TurnLeftWall();
  
  
  forward(LeftSlowForward, RightSlowForward);
  while(!bumpTop)
  {
    forwardLidar(LeftSlowForward, RightSlowForward);
  }
  brake();
  
  servoArm.write(trapDown);
  delay(2000);
  servoArm.write(trapUp);
  delay(500);
  
  //reverse
  forwardLidar(RightSlowForward, LeftSlowForward);
  
  delay(2000);
  //180
  forward(RightFastForward, RightFastForward);
  delay(2500); 
  
  forward(1294, RightSlowForward);
  while(!bumpTop);
  {
    forwardLidar(LeftSlowForward, RightSlowForward);
  }

  //Turn 90
  forward(RightMediumForward, LeftMediumForward);
  delay(800);
  
  forward(LeftMediumForward, LeftMediumForward);
  delay(355);

// 4 wheel drive
  servoClampGripLeft.write(ClampLeftGripForward);
  servoClampGripRight.write(155);
  delay(1000);

  servoClampHingeLeft.write(ClampLeftHingeDown);
  servoClampHingeRight.write(ClampRightHingeDown);
  delay(1000);
  
  forward(LeftFastForward, RightFastForward);
  
  //Grip ramp
  while(!onRamp(0.6));
  
  brake();
  
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
  
  //Climb
  ClimbRamp();
  
  forward(LeftMediumForward, RightFastForward);
  delay(2000); 
   
  //Turn 90
  forward(LeftFastForward, LeftFastForward);   
  delay(355);
  
  //drive to base
  // Front Wheel driving 
  GetOffBase();
  forward(LeftSlowForward, RightSlowForward);   
   while(!bumpTop);
   brake();
  
   //redploy trap 
   servoArm.write(trapDown);
   
   //WE WIN
   while(1);
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
