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
#define ROTATION 90 //or -90

// Isolated Test Defines (toggle ON.OFF) -> For Phase B
//#define MOVE_CENTRES
//#define SPOT_TURN
//#define STRAIGHT
//#define SQUARE_WAVE

#define WHEEL_CIRCUM 240.0 //in mm
#define COUNTS_PER_REV 720.0 //calibration = 1 count = 0.5 degrees

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

units::percentage mFullSpeed(75.0);
units::percentage mHalfSpeed(30.0);

// State Machine
#define MODE_IDLE   (0x0000)                  
#define MODE_DRIVE  (0x0001)                 
#define MODE_TEST   (0x0002) 
int system_mode = MODE_IDLE;

void setup() {

  Serial.begin(115200);
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel();  
  delay(100);
}

// Functions for motor control and manoeuvres
void robotForward()
{
  right_motor::forward(mHalfSpeed);
  left_motor::forward(mHalfSpeed);  
}

void robotBackward()
{
  right_motor::backward(mHalfSpeed);
  left_motor::backward(mHalfSpeed);  
}

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
}

void loop()
{
    switch(system_mode)
    {

      case (MODE_IDLE):
      
        //Start LED Sequence
        if ((millis() - led_timer) > 2000)
        {
          led_timer = millis();
          
          //Green OFF and Red ON
          statusGreen::write(logic_level::low);
          statusRed::write(logic_level::high);     
          system_mode = MODE_DRIVE; //change to state of driving
          delay(15);       
        }
        
        //Green ON and Red OFF
        statusGreen::write(logic_level::high);
        statusRed::write(logic_level::low);

        break;

      case (MODE_DRIVE): 
        robotForward();
        if (millis() - motor_timer > 5000)
        {
          motor_timer = millis();
          robotStop();
        }
       
        break;

      case (MODE_TEST):
      
        Serial.println("TESTING");
//        Serial.print("rEncoder: "); Serial.print(-eCountR);
//        Serial.print("lEncoder: "); Serial.print(eCountL);
//        Serial.print(" | rWheel(mm): "); Serial.println(abs(-(eCountR/COUNTS_PER_REV)*WHEEL_CIRCUM)); 
//        Serial.print(" | lWheel(mm): "); Serial.println(abs((eCountL/COUNTS_PER_REV)*WHEEL_CIRCUM)); 
        break;

      default:
        break;      
    }

  //2. The robot starts at the centre of a cell and moves forward to the centre of the next cell.
  //The cells are not surrounded by walls.
  
  #ifdef MOVE_CENTRES
    
  #endif
   
  //3. The robot starts at the centre of a cell and rotates for an angle (90deg or -90deg) specified by the
  //demonstrator via Bluetooth (the demonstrator inputs 90 means the robot should turn left for 90deg; the
  //demonstrator inputs –90 means the robot should turn right for 90deg) while remaining at the same cell.
  //The cell is not surrounded by walls.

  #ifdef SPOT_TURN
               
  #endif

  //4. The robot starts at the centre of a cell and moves forward in a straight line for 7 cells
  //(including the starting cell) without hitting the walls. (

  #ifdef STRAIGHT
    
  #endif

  //5. The robot starts at the centre of a cell and moves along a square wave path
  //for 10 cells (including the starting cell) without hitting the walls. (

  #ifdef SQUARE_WAVE

  #endif  
    
  delay(50);
}    
