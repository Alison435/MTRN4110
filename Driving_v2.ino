// MTRN4110 Phase B : DRIVING
// Created by Jonathan Loong (z5014751) 
// Date: 26/6/2019

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1500.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define CELL_LENGTH         250.0 // mm
#define STRAIGHT_DISTANCE   225.0

// Isolated Test Defines (toggle ON.OFF) -> For Phase B
#define TEST_MOVE_CENTRES
#define TEST_SPOT_TURN
#define TEST_STRAIGHT
#define TEST_SQUARE_WAVE
#define TEST_AUTO

typedef enum robot_motion
{
  ROBOT_FORWARD,
  ROBOT_LEFT,
  ROBOT_RIGHT,  
} ROBOT_MOVE;

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
    motorControl(1,0,rightPWM,leftPWM);

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

void robotTurn(int directionVal)
{

  float setSpeedPerc = 30.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 0)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV/3.20)
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

// Timing for hardware tests
unsigned long motor_duration = 5000;
unsigned long motor_timer = 0.0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level set LOW as default (to be toggled)
char drivemode;

void setup() {

  Serial.begin(9600);  
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  pinMode(13,OUTPUT); 
  delay(100);
}

void loop()
{
  
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high);  
  
  // Using Bluetooth Module (COM31)
  if (Serial.available() > 0)   
  {
    drivemode = Serial.read();  

    switch (drivemode)
    {
          
      case '1':
        startLEDSequence();     
        mode_centres();
        break;

      case '2':
        startLEDSequence();    
        mode_spot_turn();
        break;
            
      case '3':
        startLEDSequence();   
        mode_straight();
        break;
            
      case '4':
        startLEDSequence();
        mode_square_wave();
        break;

      case 'a':
        startLEDSequence();
        mode_auto();
        break;

      default:
        break;           
     }
   }     
}        

void mode_centres()
{
  //1. The robot starts at the centre of a cell and moves forward to the centre of the next cell.
  //The cells are not surrounded by walls.
        
  #ifdef TEST_MOVE_CENTRES
    //Serial.println("moving to next cell");
    //motorControl(1,0,25.0,25.0); //.debug
    robotForward(225.0,30.0);
    statusGreen::write(logic_level::low);
    statusRed::write(logic_level::high); 
    delay(1000);         
  #endif
  
}              
 
void mode_spot_turn()
{

   //2. The robot starts at the centre of a cell and rotates for an angle (90deg or -90deg) specified by the
   //demonstrator via Bluetooth (the demonstrator inputs 90 means the robot should turn left for 90deg; the
   //demonstrator inputs –90 means the robot should turn right for 90deg) while remaining at the same cell.
   //The cell is not surrounded by walls.
      
   #ifdef TEST_SPOT_TURN
      
     //Read in from Bluetooth
     //90 -> CCW (LEFT)
     //-90 -> CW (RIGHT)
          
     // Parameters for Bluetooth Input
          char rotArray[5];
          int counter = 0;
          int rotFlag = 0;
          int input_complete = 0;
          int newCounter = 0;

     //Set Serial monitor to new line
     Serial.println("Awaiting Input from Bluetooth");          
      
     while(input_complete == 0)
     {
        while (Serial.available() > 0)
        {             
          byte incomingByte = Serial.read();
    
          switch (incomingByte)
          {        
            case '\n':   // end of text
              if (newCounter == 0)
              {
                newCounter = 1;
                break;                
              }
              rotArray [counter] = 0;  // terminating null byte                                     
              input_complete = 1;
              break;
                
            case '\r':   // discard carriage return
              break;
      
            case '-':
              rotFlag = 1;
              rotArray [counter] = '-';    
              break;              
                
            default:
              // keep adding if not full ... allow for terminating null byte
              if (counter < (7))
                rotArray [counter++] = incomingByte;
              break;        
           }
       }
    }       

    //Serial.println("INPUT RECEIVED");
              
   if (rotFlag == 0)
   {
    led::write(logic_level::high);
    robotTurn(0); //TURN LEFT            
   }
   else if (rotFlag == 1) //-ve sign detected in input
   {
    led::write(logic_level::high);
    robotTurn(1); //TURN RIGHT
   }
 
   statusGreen::write(logic_level::low);
   statusRed::write(logic_level::high); 
                     
#endif  
}       

void mode_straight()
{
  //3. The robot starts at the centre of a cell and moves forward in a straight line for 7 cells
  //(including the starting cell) without hitting the walls. (
      
  Serial.println("STRAIGHT");
  delay(50);
  int j = 0;
  for (j = 0; j < 7; j++)
  {
    robotForward(STRAIGHT_DISTANCE,30.0);
  }
          
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 
  delay(1000);  
}

void mode_square_wave()
{

//4. The robot starts at the centre of a cell and moves along a square wave path
//for 10 cells (including the starting cell) without hitting the walls.
      
#ifdef TEST_SQUARE_WAVE

  //slslsrsrslslsrsrs

  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(0);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(0);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(1);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(1);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(0);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(0);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(1);
  robotForward(STRAIGHT_DISTANCE,30.0);
  robotTurn(1);
  robotForward(STRAIGHT_DISTANCE,30.0);

//  Serial.println("SQUARE");
//  robotTurn(0); //ccw
//  delay(1000);
//  robotTurn(1); //cw
  delay(500);
            
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 
    
#endif          
}

void mode_auto()
{

#ifdef TEST_AUTO
  
  //Take in array of commands (5 cells towards goal)
  //Turn into physical motion
  //eg: receive array of "^^>^<^"
  
  #define MAX_INPUT 7
  char robotMovements[MAX_INPUT];
  char moveChar;
  int move_i = 0;
  int commandFinished = 0;
  String command;
  int newCount = 0;
  
  Serial.println("Awaiting commands");
  while (commandFinished == 0)
  {
    while (Serial.available () > 0) //change to Serial1
    {
      moveChar = Serial.read();
      switch (moveChar)
      {  
        case('\n'):
          if (newCount == 0)
          {
            newCount = 1;
            break;                
          }
          robotMovements[move_i] = 0; //null terminator
          commandFinished = 1;
          //Serial.println(robotMovements);
          break;

        case '\r':   // discard carriage return
          break;
                    
        default:
          if (move_i < MAX_INPUT - 1)
            robotMovements[move_i++] = moveChar; //fill in commandArry 
          break;
       }
   }  
  }

  Serial.println("--COMMANDS READY--");
          
  // Iterate through command Array
  for (int i = 0; i < MAX_INPUT; i++)
  {
    char j = robotMovements[i];
      
    switch (j)
    {  
      case('^'):
        Serial.println("FORWARDS");
        robotForward(STRAIGHT_DISTANCE,30.0);
        delay(15);        
        break;
    
      case('>'):
        Serial.println("RIGHT");
        robotTurn(1);
        delay(15);        
        break;
    
      case('<'):        
        Serial.println("LEFT");
        robotTurn(0);
        delay(15);        
        break;
                
     default:
       break;
    }
    delay(100);             
  }

  // ROBOT is at goal cell:
  // Green LED OFF and red LED ON
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 

#endif 
}  

        
