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
#define MOVE_CENTRES
//#define SPOT_TURN
//#define STRAIGHT
//#define SQUARE_WAVE

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
//#define ROBOT_FORWARD   (pwm) {ROBOT_FORWARD,pwm}
//#define ROBOT_LEFT      (pwm) {ROBOT_LEFT,pwm}
//#define ROBOT_RIGHT     (pwm) {ROBOT_RIGHT,pwm}
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

// Timing for hardware tests
unsigned long motor_duration = 5000;
unsigned long motor_timer = 0.0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level set LOW as default (to be toggled)

units::percentage mFullSpeed(50.0);
units::percentage mHalfSpeed(30.0);

// State Machine
#define MODE_IDLE   (0x0000)                  
#define MODE_DRIVE  (0x0001)
#define MODE_AUTO   (0x0002)      
#define MODE_OFF    (0x0003)           
                 
int system_mode = MODE_IDLE;
char drivemode;

void setup() {

  Serial.begin(115200);
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel();  
  delay(100);
}

// Functions for motor control and turning
// Implementing Proportional Controller for differential drive system

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
    left_motor::backward(mHalfSpeed);
    right_motor::forward(mHalfSpeed);
  }

  //-ve so turn to right (CW)
  left_motor::forward(mHalfSpeed);
  right_motor::backward(mHalfSpeed);
}

void loop()
{
  if (Serial.available() > 0) drivemode = Serial.read();
  else drivemode = -1;
    
    switch(system_mode)
    {

      case (MODE_IDLE):

        //Green OFF and Red ON
        statusGreen::write(logic_level::low);
        statusRed::write(logic_level::high);
      
        //Start LED Sequence
        if ((millis() - led_timer) > 5000)
        {
          led_timer = millis();
          
          //Green ON and Red OFF
          statusGreen::write(logic_level::high);
          statusRed::write(logic_level::low);     
          system_mode = MODE_DRIVE; //change to state of driving
          delay(100);       
        }
        
        break;

      case (MODE_DRIVE): 

        if (drivemode == 'a')
        {
          system_mode = MODE_AUTO; //from computer vision section
        }
        
        //1. The robot starts at the centre of a cell and moves forward to the centre of the next cell.
        //The cells are not surrounded by walls.
        
        #ifdef MOVE_CENTRES
          Serial.println("moving to next cell");
        #endif
         
        //2. The robot starts at the centre of a cell and rotates for an angle (90deg or -90deg) specified by the
        //demonstrator via Bluetooth (the demonstrator inputs 90 means the robot should turn left for 90deg; the
        //demonstrator inputs –90 means the robot should turn right for 90deg) while remaining at the same cell.
        //The cell is not surrounded by walls.
      
        #ifdef SPOT_TURN
      
          //Read in from Bluetooth
          desiredRotation = 90;
                     
        #endif
      
        //3. The robot starts at the centre of a cell and moves forward in a straight line for 7 cells
        //(including the starting cell) without hitting the walls. (
      
        #ifdef STRAIGHT
          
        #endif
      
        //4. The robot starts at the centre of a cell and moves along a square wave path
        //for 10 cells (including the starting cell) without hitting the walls. (
      
        #ifdef SQUARE_WAVE
      
        #endif        
        break;

      case (MODE_AUTO):
        //Serial input from Vision section to start
        Serial.println("In Auto Mode");
        
        if (drivemode == 'e')
        {
          system_mode = MODE_OFF;
        }
        break;

     case (MODE_OFF):

        // Goal found -> Green OFF and Red ON
        statusGreen::write(logic_level::low);
        statusRed::write(logic_level::high);          
        break;

      default:
        break;      
    } 
    
  delay(25);
}    
