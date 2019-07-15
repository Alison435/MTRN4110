// Combining Driving and Exploration 
// TODO: include logic for wall detection and move decisions (EXPLORATION)

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>
#include <Wire.h>
#include <Sensors.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1500.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define CELL_LENGTH         250.0 // mm
#define STRAIGHT_DISTANCE   200.0

// Test Defines
//#define USING_LIDAR
//#define USING_MOTORS

int trigPin = 11;  //change for mega pin 32
int echoPin = 12;  //change for mega pin 33
int lidarone = 30; //shutdown pins for lidar on mega - 30
int lidartwo = 31; //mega - 31

bool robot_start = false; //will be set to true once LED sequence is finished
int movement_num = 0;

// Encoder variables for Phase A
volatile int eCountR = 0; 
volatile int eCountL = 0; 
volatile byte pinAcountR;
volatile byte pinAcountL;
boolean DirectionR = true; // Rotation direction for right motor
boolean DirectionL = true; // Rotation direction for left motor

void setUpUltrasonic()
{
    //Ultrasonic set up
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    //Serial.println("Ultrasonic initialised");
}

void setUpLidars()
{
  #ifdef USING_LIDAR
     
    //Setting address for lidars
    pinMode(lidarone,OUTPUT);
    pinMode(lidartwo,OUTPUT);
    digitalWrite(lidartwo, HIGH);//enable lidar two
    digitalWrite(lidarone, LOW); //shut down lidar one before chaning address
    delay(100);
    lidarTwo.setAddress(0x30);
    delay(10);
    digitalWrite(lidarone, HIGH);//enable lidar one 
    
    lidarTwo.init();
    Serial.println("Lidar Two initialised");
    lidarTwo.configureDefault();
    lidarTwo.setTimeout(500);
    
    lidarOne.init();              //lidar one uses default address
    Serial.println("Lidar One initialised");
    lidarOne.configureDefault();
    lidarOne.setTimeout(500);
    
 #endif USING_LIDAR

}

float ultrasonicdist() {
  long duration;
  float distance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (float(duration)/(29.155*2));
  distance = distance*10;

  if (distance > 1000) {  //setting max distance at 1000
    distance = 1000;
  }
  return distance;
}

// Motor variables (to match hardware_definitions.h)
int leftEnableDC = 10; //pwm
int rightEnableDC = 11; //pwm 
int in1 = 9; //digital
int in2 = 8; //digital
int in3 = 12; //digital
int in4 = 13; //digital

//function for the motor controller
void motorControl(int pin1,int pin2,float percRight,float percLeft)
{  
  if (pin1 == 1) {
    analogWrite(leftEnableDC,(percLeft/100)*255);
    digitalWrite(in1,HIGH); //forward 
    digitalWrite(in2,LOW);  
  }
  else
  {
    analogWrite(leftEnableDC,(percLeft/100)*255);
    digitalWrite(in1,LOW); 
    digitalWrite(in2,HIGH);
  }

  if (pin2 == 1) {
    analogWrite(rightEnableDC,(percRight/100)*255);
    digitalWrite(in3,HIGH); 
    digitalWrite(in4,LOW);  
  }
  else
  {
    analogWrite(rightEnableDC,(percRight/100)*255);
    digitalWrite(in3,LOW); 
    digitalWrite(in4,HIGH);  
  }
}

void robotForward(float distance, float setSpeedPerc)
{
  long targetCount;
  float numRev;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  float leftPWM = setSpeedPerc;
  float rightPWM = setSpeedPerc;

  // variables for tracking the left and right encoder counts
  long prevlCount, prevrCount;
  
  // variable used to offset motor power on right vs left to keep straight.
  double offset = 0.35;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * COUNT_PER_REV;    // calculate the target count
 
  // Reset encoders
  eCountL = 0;
  eCountR = 0;
  delay(50);

  motorControl(1,0,rightPWM,leftPWM);

  while (abs(eCountR) < targetCount)
  {
    motorControl(1,0,rightPWM,leftPWM+0.5);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (abs(eCountL) - prevlCount);
    rDiff = (abs(eCountR) - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = -eCountL;
    prevrCount = -eCountR;

    // if left is faster than the right, slow down the left / speed up right
    if (lDiff > rDiff) 
    {
      leftPWM -= offset;
      rightPWM += offset;
    }
    // if right is faster than the left, speed up the left / slow down right
    else if (lDiff < rDiff) 
    {
      leftPWM += offset;  
      rightPWM -= offset;    }
    delay(50);  // short delay to give motors a chance to respond.
  }
  
  robotStop();
  delay(1000);
}

// Functions for motor control and turning
// Implementing Proportional Controller for differential drive system

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
}

void resetAllEncoders()
{
  eCountL = 0;
  eCountR = 0;
  delay(10);
}

void robotTurn(int directionVal)
{

  float setSpeedPerc = 30.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 0)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV/3.10)
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(500);
  }

  else //directionVal is 1
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV/3.50)
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);  
    }

  resetEncoderR();
  resetEncoderL(); 
  robotStop();    
  delay(1000); 
  }  
}

// ISR for encoder interrupt at Right Motor
void encoderCountR()
{
    encoder_count LstateR = int(pins::right_encoder_a::read());

    if((pinAcountR == LOW) && LstateR == HIGH)
    {
        encoder_count encoder_bR = encoder_count(pins::right_encoder_b::read());

        if(encoder_bR == LOW && DirectionR)
        {
          DirectionR = false; //Reverse

        }
        else if(encoder_bR == HIGH && !DirectionR)
        {
          DirectionR = true;  //Forward
        }
    }

    pinAcountR = LstateR;

    if(!DirectionR)  eCountR++;
    else  eCountR--;
}

// ISR for encoder interrupt at Left Motor
void encoderCountL()
{
    encoder_count LstateL = encoder_count(pins::left_encoder_a::read());

    if((pinAcountL == LOW) && LstateL == HIGH)
    {
        encoder_count encoder_bL = encoder_count(pins::left_encoder_b::read());

        if(encoder_bL == LOW && DirectionL)
        {
          DirectionL = false; //Reverse

        }
        else if(encoder_bL == HIGH && !DirectionL)
        {
          DirectionL = true;  //Forward
        }
    }

    pinAcountL = LstateL;

    if(!DirectionL)  eCountL++;
    else  eCountL--;
}

void resetEncoderR()
{
  eCountR = 0;
}

void resetEncoderL()
{
  eCountL = 0;
}

void setupDigitalPins()
{
  led::config_io_mode(io_mode::output);
  statusRed::config_io_mode(io_mode::output);
  statusGreen::config_io_mode(io_mode::output);
}

void setupMotor()
{  
  right_motor::enable();
  left_motor::enable();
}

void setupEncoderWheel()
{
  left_encoder::enable();
  right_encoder::enable();
  
  // Pure Arduino Interrupt Setup
  attachInterrupt(digitalPinToInterrupt(3), encoderCountR, CHANGE); //ISR for Right Motor
  attachInterrupt(digitalPinToInterrupt(2), encoderCountL, CHANGE); //ISR for Left Motor 
}

void startLEDSequence()
{
  //Green ON and Red OFF
  statusGreen::write(logic_level::high);
  statusRed::write(logic_level::low);   
}

void robotLeft()
{
  resetAllEncoders();   
  robotTurn(0); //turn CCW on spot
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,30.0); //go forward on cell
  resetAllEncoders();  
}

void robotRight()
{
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,30.0); //go forward one cell
  resetAllEncoders();  
}

// Timing for hardware tests
unsigned long explore_test = 0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;
char drivemode;

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);  
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  setUpLidars();
  setUpUltrasonic();
  pinMode(13,OUTPUT); 
  delay(100);
}

// State Machine
#define MODE_OFF 1
#define MODE_EXPLORE 2
#define MODE_GOAL 3
int system_mode = MODE_OFF;

float distance;  

void loop()
{

  // Start LED sequence:
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 
  resetAllEncoders();
  
  // Using Bluetooth Module (COM31) on TX and RX (Serial Port 0)
  // Manual testing of exploration logic and actuation
  if (Serial1.available() > 0)   
  {
    drivemode = Serial1.read();  

    // Physical motion after each decision is made
    // TODO: synthesise actuation with logic autonomously
    switch (drivemode)
    {
          
      case 's':
        startLEDSequence();     
        robotForward(STRAIGHT_DISTANCE,30.0); //go forward one cell         
        break;

      case 'l':
        startLEDSequence(); 
        robotLeft();
        break;
            
      case 'r':
        startLEDSequence();   
        robotRight();
        break;

      //reverse if dead end found
      case 'b':
        startLEDSequence();
        resetAllEncoders(); 
        robotTurn(0); //turn CCW on spot
        resetAllEncoders();
        delay(15);
        robotTurn(0); //turn CCW on spot;
        resetAllEncoders();
        break;
        
      default:
        break;           
     }

  //Check which direction to go next
  //Lidar/Ultrasonic -> forward, reverse, right or left
     
   }     
}               
