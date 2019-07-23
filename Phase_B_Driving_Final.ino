// MTRN4110 Phase B : DRIVING
// Created by Jonathan Loong (z5014751) 
// Date: 26/6/2019

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>
#include <Wire.h>
#include <Sensors.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1400.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define CELL_LENGTH         250.0 // mm
#define STRAIGHT_DISTANCE   200.0 // mm
#define LIDAR_MAX_SETPOINT  100.0 // mm

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

bool using_lidar = false;

void setUpLidars()
{
  Wire.begin();
  pinMode(LIDARONE,OUTPUT);
  pinMode(LIDARTWO,OUTPUT);
  digitalWrite(LIDARTWO, HIGH);//enable lidar two
  digitalWrite(LIDARONE, LOW); //shut down lidar one before chaning address
  delay(100);
  lidarTwo.setAddress(0x30);
  delay(10);
  digitalWrite(LIDARONE, HIGH);//enable lidar one 
  
  lidarTwo.init();
  Serial.println("Lidar Two initialised");
  lidarTwo.configureDefault();
  lidarTwo.setTimeout(500);
  
  lidarOne.init();              //lidar one uses default address
  Serial.println("Lidar One initialised");
  lidarOne.configureDefault();
  lidarOne.setTimeout(500);

}

// Control Parameters
double error_P_lidar = 0.0;
double error_P_encoder = 0.0;

double error_D_lidar = 0.0;
double error_D_encoder = 0.0;

double prev_error_encoder = 0.0;
double prev_error_lidar = 0.0;

double totalError = 0.0;
double sumError = 0.0;

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

//-------------------PID Parameters---------------------//
// Constants for Lidar
double K_p_lidar = 0.15;
double K_d_lidar = 0.05;

// Constants for Encoder
double K_p_encoder = 0.5;
double K_d_encoder = 0.1;
double K_i_encoder = 0.2;
//------------------------------------------------------//

void robotForward(float distance, float setSpeedPerc)
{
  long targetCount;
  float numRev;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  float leftPWM = setSpeedPerc;
  float rightPWM = setSpeedPerc;

  float prevlCount, prevrCount;
 
  // Bias - variable used to offset motor power on right vs left to keep straight.
  double offset = 0.12;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * (COUNT_PER_REV);    // calculate the target count
 
  // Reset encoders
  eCountL = 0;
  eCountR = 0;
  delay(15);

  motorControl(1,0,rightPWM,leftPWM);

  while (abs(eCountR) < targetCount)
  {
    motorControl(1,0,rightPWM,leftPWM + offset);

    // Error count from encoder 'ticks'
    lDiff = (abs(eCountL) - prevlCount);
    rDiff = (abs(eCountR) - prevrCount);
  
    // Store previous count  
    prevlCount = -eCountL;
    prevrCount = -eCountR;

    // Have walls on either side (Part 3 of Phase B)
    // L1 = 65. L2 = 45. Bias = 15
    if (using_lidar == true)
    {
     if (lidarTwo.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT && lidarOne.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT) //10-110
      {
        error_P_lidar = lidarOne.readRangeSingleMillimeters() - lidarTwo.readRangeSingleMillimeters() - 15.0;
        error_D_lidar = error_P_lidar - prev_error_lidar;
  
        //K_p and K_d (for lidar)  
        totalError = (K_p_lidar * error_P_lidar) + (K_d_lidar * error_D_lidar);
        prev_error_lidar = error_P_lidar;
  
        leftPWM -= totalError;
        rightPWM += totalError;    
      }     
    }
    // If no wall on either side
    else
    {
      double e_error = (lDiff - rDiff); //encoder error between left and right wheel
      sumError += e_error;      
      double dError = (lDiff - rDiff) - prev_error_encoder;

      totalError = (K_p_encoder * e_error) + (K_d_encoder * dError) + 
      (K_i_encoder * sumError);

      leftPWM += totalError;  
      rightPWM -= totalError;

      prev_error_lidar = e_error; //store previous error for next cycle
    }
  }
  robotStop();
  delay(10);
  resetAllEncoders();
}

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
  delay(15);
}

void robotTurn(int directionVal)
{

  float setSpeedPerc = 25.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 1)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV/3.50)
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(200);
  }

  else //directionVal is 0
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV/3.12)
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);  
    }

  resetEncoderR();
  resetEncoderL(); 
  robotStop();    
  delay(200); 
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

void resetAllEncoders()
{
  eCountL = 0;
  eCountR = 0;
  delay(10);
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
unsigned long motor_timer = 0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level set LOW as default (to be toggled)
char drivemode;

void setup() {
  
  Serial3.begin(9600);  
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  setUpLidars();
  pinMode(13,OUTPUT); 
  delay(100);
}

void loop()
{
  
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high);  
  
  // Using Bluetooth Module (COM31)
  if (Serial3.available() > 0)   
  {
    drivemode = Serial3.read();  

    switch (drivemode)
    {
          
      case '1':
        startLEDSequence(); 
        using_lidar = false;
        resetAllEncoders();   
        mode_centres();
        break;

      case '2':
        startLEDSequence(); 
        using_lidar = false;
        resetAllEncoders();   
        mode_spot_turn();
        break;
            
      case '3':
        startLEDSequence();
        using_lidar = true;
        resetAllEncoders();      
        mode_straight();
        break;
            
      case '4':
        startLEDSequence();
        using_lidar = false;
        resetAllEncoders();   
        mode_square_wave();
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
    resetAllEncoders();
    delay(15);
    robotForward(STRAIGHT_DISTANCE,20.0);
    resetAllEncoders();   
    statusGreen::write(logic_level::low);
    statusRed::write(logic_level::high);   
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
        while (Serial3.available() > 0)
        {             
          byte incomingByte = Serial3.read();
    
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
    resetAllEncoders();   
    robotTurn(0); //TURN LEFT            
   }
   else if (rotFlag == 1) //-ve sign detected in input
   {
    led::write(logic_level::high);
    resetAllEncoders();   
    robotTurn(1); //TURN RIGHT
   }
 
   statusGreen::write(logic_level::low);
   statusRed::write(logic_level::high); 
   resetAllEncoders();   

                     
#endif  
}       

void mode_straight()
{
  //3. The robot starts at the centre of a cell and moves forward in a straight line for 7 cells
  //(including the starting cell) without hitting the walls. (
      
  Serial.println("STRAIGHT");

  for (int j = 0; j < 6; j++)
  {
    resetAllEncoders();
    robotForward(STRAIGHT_DISTANCE,25.0);
    robotStop();
    resetAllEncoders();
    delay(15);
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

  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(0);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(0);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(1);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(1);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(0);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(0);  
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(1);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
  resetAllEncoders();
  robotTurn(1);
  delay(25);
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,20.0);
  delay(25);
            
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 
    
#endif          
}


        
