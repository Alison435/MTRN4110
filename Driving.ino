// MTRN4110 Phase B : DRIVING
// Created by Jonathan Loong (z5014751) 
// Date: 26/6/2019

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// Demonstrator defined turn for Move #2
int desiredRotation; //90 or -90 input at serial (through Bluetooth)

// Motor/Wheel parameters
#define COUNT_PER_REV       1920  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define GEAR                120.0 // 120:1
#define CELL_LENGTH         250.0 // mm

// Isolated Test Defines (toggle ON.OFF) -> For Phase B
#define TEST_MOVE_CENTRES
//#define TEST_SPOT_TURN
//#define TEST_STRAIGHT
//#define TEST_SQUARE_WAVE

//typedef enum robot_motion
//{
//  ROBOT_FORWARD,
//  ROBOT_LEFT,
//  ROBOT_RIGHT,  
//} ROBOT_MOVE;
//
//typedef struct robot_movement
//{
//  ROBOT_MOVE  driveAction;
//  double      setPWM;
//} COMMAND_SEQ
//
//
//COMMAND_SEQ robotMovements[] = 
//{
//
//}
//
//// Pointer to sequence for robot moves
//COMMAND_SEQ *move_ref;

bool robot_start = false; //will be set to true once LED sequence is finished
int movement_num = 0;

//move_ref = &robotMovements[movement_num]

// Encoder variables for Phase A
volatile int eCountR = 0; 
volatile int eCountL = 0; 
volatile byte pinAcountR;
volatile byte pinAcountL;
boolean DirectionR = true; // Rotation direction for right motor
boolean DirectionL = true; // Rotation direction for left motor

// Motor variables (to match hardware_definitions.h)
int leftEnableDC = 10; //pwm
int rightEnableDC = 11; //pwm 
int in1 = 9; //digital
int in2 = 8; //digital
int in3 = 12; //digital
int in4 = 13; //digital

//function for the motor controller
void motor(int pin1,int pin2,int percRight,int percLeft)
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

// Timing for hardware tests
unsigned long motor_duration = 5000;
unsigned long motor_timer = 0.0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level set LOW as default (to be toggled)

// State Machine
#define MODE_IDLE         (0x0000)                  
#define MODE_CENTRES      (0x0001)
#define MODE_SPOT_TURN    (0x0002)  
#define MODE_STRAIGHT     (0x0003)
#define MODE_SQUARE_WAVE  (0x0004)  
#define MODE_AUTO         (0x0005)  
#define MODE_OFF          (0x0006)           
                 
int system_mode = MODE_IDLE;
char drivemode;

void setup() {

  Serial.begin(115200);
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  pinMode(13,OUTPUT); 
  delay(100);
}

// Functions for motor control and turning
// Implementing Proportional Controller for differential drive system

// Forward for distance at desired power
void robotForward()
{
  
}

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
}

void robotTurn()
{

  if (desiredRotation > 0) //+ve means turn to left (CCW)
  {
    //Right + and Left -
    //Encoder count = 90
  }

  //-ve so turn to right (CW)
  //Right - and Left +
  //Encoder count = 90

  
}

void loop()
{
  // Using Bluetooth Module (COMX)
  if (Serial.available() > 0) drivemode = Serial.read();
  else drivemode = -1;
    
    switch(system_mode)
    {

      case (MODE_IDLE):

        //Green OFF and Red ON
        statusGreen::write(logic_level::low);
        statusRed::write(logic_level::high);

        switch (drivemode)
        {
  
          case ('a'):
            startLEDSequence(); 
            system_mode = MODE_AUTO;
            break;

          case ('1'):
            startLEDSequence();     
            system_mode = MODE_CENTRES;
            break;

          case ('2'):
            startLEDSequence();    
            system_mode = MODE_SPOT_TURN;
            break;
            
          case ('3'):
            startLEDSequence();   
            system_mode = MODE_STRAIGHT;
            break;
            
          case ('4'):
            startLEDSequence();  
            system_mode = MODE_SQUARE_WAVE;
            break;

          default:
            break;
           
        }
                
        break;

      case (MODE_CENTRES): 
        
        //1. The robot starts at the centre of a cell and moves forward to the centre of the next cell.
        //The cells are not surrounded by walls.
        
        #ifdef TEST_MOVE_CENTRES
          Serial.println("moving to next cell");
          //robotForward();
        #endif
        break;
        
      case (MODE_SPOT_TURN):
        
        //2. The robot starts at the centre of a cell and rotates for an angle (90deg or -90deg) specified by the
        //demonstrator via Bluetooth (the demonstrator inputs 90 means the robot should turn left for 90deg; the
        //demonstrator inputs –90 means the robot should turn right for 90deg) while remaining at the same cell.
        //The cell is not surrounded by walls.
      
        #ifdef TEST_SPOT_TURN
      
          //Read in from Bluetooth
          desiredRotation = 90;
                     
        #endif
      
        break;
         
     case (MODE_STRAIGHT):

        //3. The robot starts at the centre of a cell and moves forward in a straight line for 7 cells
        //(including the starting cell) without hitting the walls. (
      
        #ifdef TEST_STRAIGHT
          
        #endif
        
        break;        
      
      case (MODE_SQUARE_WAVE):
      
        //4. The robot starts at the centre of a cell and moves along a square wave path
        //for 10 cells (including the starting cell) without hitting the walls. (
      
        #ifdef TEST_SQUARE_WAVE
      
        #endif        
        break;

      case (MODE_AUTO):

        // debug to see if in AUTO mode (after CV)
        led::write(logic_level::low);
        
        if (millis() - led_timer > 2000)
        {
          led_timer = millis();
          led::write(logic_level::high);
        }
        
        if (drivemode == 'e')
        {
          system_mode = MODE_OFF;
        }
        
        break;

     case (MODE_OFF):

        // Goal found -> Green OFF and Red ON
        statusGreen::write(logic_level::low);
        statusRed::write(logic_level::high);     
        right_motor::stop();
        left_motor::stop();     
        break;

      default:
        break;      
    } 
    
  delay(25);
}    
