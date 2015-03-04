#include <Servo.h> 
#include <Wire.h>
#include <LSM303.h>
 
LSM303 compass;
float heading;
float initialHeading;
float gravity;
float initialGravity;
float onRampGravity;
 
int servoLeftPin = 5;
int servoRightPin = 6;
Servo servoLeft;  // create servo object to control a servo
Servo servoRight;  // create servo object to control a servo

int servoArmPin = 4;
Servo servoArm;

int servoFlangePin = 3;
Servo servoFlange;

int bumpForwardPin = 7;
volatile boolean bumpForward = false; // Triggered true by boolean. 

int IRLeftPin = 8;
int IRRightPin = 9;

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
  if(heading < initial Heading)
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

void moveUntilBump()
{
  forward();
  while(!(bumpForward)){}
  bumpForward = false;  
}

void setup() 
{ 
  attachInterrupt(0, ISR_BUMP, RISING); // Note- can be LOW, CHANGE, RISING, or FALLING
    
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  
  pinMode(IRLeftPin, INPUT);
  pinMode(IRRightPin, INPUT);
  pinMode(bumpForwardPin, INPUT);  
  
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
  
  // Turn 90 degrees
  turn(180.0);
  
  
  
  // Scan
  // If probable height then take next few readings to see if they are the same.
  int start, ending;
  for(int i = 0; i < 180; i++)
  {
    // Scan for heights.
    // record range of degrees that have select heights first the start then the ending.
  }
  // Head in direction of the middle of the clump of valuable readings.
  double dir_base = ((start + ending) /2.0)+90.0;
  turn(dir_base);
  
  // If none then head forward a little bit.
  moveUntilBump();
  servoArm.write(0);  // Swat down
  delay(1000); // Wait for arm  
  servoArm.write(180);  // Swat up
  
  reverse(); // reverse a bit to back away from base.
  delay(200);
  turn(0.0);
  
  moveUntilBump();
  // Turn until heading is lined up with ramp on return.
  turn(270.0);
  
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

void ISR_BUMP()
{
  // Double check that arduinon automaticaly resets the register for me.
  bumpForward = true;
}
