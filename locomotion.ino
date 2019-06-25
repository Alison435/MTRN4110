// MTRN4110 Phase A : Locomotion Hardware/Software Interfacing Test
// Created by Jonathan Loong (z5014751) 

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// ------------Demonstrator-assigned wheel angle (in degrees)---------------
#define LEFT_WHEEL_ANGLE 90
#define RIGHT_WHEEL_ANGLE -90
//-------------------------------------------------------------------------

//#define TOGGLE_LED
//#define MOTOR_1_TEST
//#define MOTOR_2_TEST
//#define L_ENCODER_TEST
#define R_ENCODER_TEST

#define PI_ 3.14
#define WHEEL_CIRCUM 240.0 //in mm
#define COUNTS_PER_REV 720.0 //calibration = 1 count = 0.5 degrees
#define ANGLE_OFFSET 15

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
  
  // Create interrupts
  // Interrupts for Encoder (right and left encoder _a)

  // Pure Arduino Interrupt Setup
  attachInterrupt(digitalPinToInterrupt(3), encoderCountR, CHANGE); //ISR for Right Motor
  attachInterrupt(digitalPinToInterrupt(2), encoderCountL, CHANGE); //ISR for Left Motor
  
  // OOP Setup 
  //attachInterrupt(digitalPinToInterrupt(2), right_encoder::count(), CHANGE); //ISR for Right Motor
  //attachInterrupt(digitalPinToInterrupt(3), left_encoder::count(), CHANGE); //ISR for Left Motor
  
}

// Timing for hardware tests
unsigned long motor_duration = 5000;
unsigned long motor_timer = 0.0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level set LOW as default (to be toggled)

// Units for Encoder + Motor + Wheel (using C++ API)
units::percentage mFullSpeed(75.0);
units::percentage mHalfSpeed(30.0);
units::percentage mtestSpeed(30.0);

void setup() {

  Serial.begin(115200);
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel();

  // Callback for interrupt
  //void (*callback)() = &encoderInterrupt;
  
  delay(100);
}

// State Machine for Test Demonstration
#define CHOOSE_TEST            (0x0000) 
#define TEST_1                 (0x0001) 
#define TEST_2                 (0x0002) 
#define TEST_3                 (0x0003) 
#define TEST_4                 (0x0004) 
#define TEST_5                 (0x0005) 

int system_mode = CHOOSE_TEST;

void loop() {

  /*
  char testNum;
  
  if (Serial.available() > 0) testNum = Serial.read();
  else testNum = -1;

  switch (system_mode)
  {
    case (CHOOSE_TEST):

      switch(testNum) //test to demonstrate entered into Serial Monitor
      {

        case(testNum == '1'):

          system_mode = TEST_1;
          break;

        case(testNum == '2'):

          system_mode = TEST_2;
          break;

        case(testNum == '3'):

          system_mode = TEST_3;
          break;

        case(testNum == '4'):

          system_mode = TEST_4;
          break;

        case(testNum == '5'):

          system_mode = TEST_5;
          break;

        default:
          break;
      }
    }
    break;

   case (TEST_1):

      if ((millis() - led_timer) > 2000)
      {
        led_timer = millis();
        
        if (statusGreen::read() == logic_level::low)
        {
          ledLogic = logic_level::high;
        }
        else
        {
          ledLogic = logic_level::low;
        }
        
        statusGreen::write(ledLogic);            

        if (test_Num == '0')
        {
          system_mode = CHOOSE_TEST;    //stop toggling LED
        }        
      }

      break;

   //2. Run motor 1 at full speed in the forward direction for 5 seconds (2), 
   //display the readings of the associated encoder on the serial monitor of Arduino (2), 
   //and display the distance travelled by the wheel

    case (TEST_2):
    
      right_motor::forward(mFullSpeed);
      //Print encoder count + distance travelled by wheel
      int rEncoderCount = right_encoder::count();
      Serial.print("rEncoder: "); Serial.println(rEncoderCount);
      //Serial.print("rWheel_dist: "); right_wheel::position();          
       
      if ((millis() - motor_timer) > motor_duration)
      {  
        motor_timer = millis();
        right_motor::stop();  
        //Serial.println("Stopped R Motor");
        delay(2000);
        system_mode = CHOOSE_TEST;
      }
      
      break;

   // 3. Run motor 2 at half speed in the backward direction for 5 seconds (2), 
   // display the readings of the associated encoder on the serial monitor of Arduino (2), 
   // and display the distance travelled by the wheel
   
    case (TEST_3):
    
       left_motor::backward(mHalfSpeed); //half speed = 50.0

      //Print encoder count + distance travelled by wheel
      Serial.print("lEncoder "); Serial.print(eCountL);
      
      //units::millimeters lDistance = left_wheel::position();    
      Serial.print(" | lWheel:dist: "); Serial.println(eCountL);        
  
      if ((millis() - motor_timer) > motor_duration)
      {  
        motor_timer = millis();
        left_motor::stop();
        Serial.println("Left Motor Stopped");  
        resetEncoderL();
        delay(3000);
      }
            
      break;

   case (TEST_4):
      
      //TEST PROGRAM
      
      system_mode = CHOOSE_TEST;
      break;

   case (TEST_5):
      
      //TEST PROGRAM
 
      system_mode = CHOOSE_TEST;
      break;
    
   default:
    break;
 }     
  */ 

  // Isolated Tests Here 
  
  //1. Digital I/O Functionality

  #ifdef TOGGLE_LED

    if ((millis() - led_timer) > 2000)
    {
      led_timer = millis();
      
      if (statusGreen::read() == logic_level::low)
      {
        ledLogic = logic_level::high;
      }
      else
      {
        ledLogic = logic_level::low;
      }      
      statusGreen::write(ledLogic);            
    }
    
  #endif

  //2. Test Motor 1 Functionality
  
   //2. Run motor 1 at full speed in the forward direction for 5 seconds (2), 
   //display the readings of the associated encoder on the serial monitor of Arduino (2), 
   //and display the distance travelled by the wheel
  
  #ifdef MOTOR_1_TEST

    right_motor::forward(mHalfSpeed);

    //Print encoder count + distance travelled by wheel
    Serial.print("rEncoder: "); Serial.print(-eCountR);

    // OOP version
    //units::millimeters rDistance = right_wheel::position(); 
    //Serial.print(" | rWheel: "); Serial.println(rDistance.count());  
       
    Serial.print(" | rWheel(mm): "); Serial.println(abs(-(eCountR/COUNTS_PER_REV)*WHEEL_CIRCUM)); 
    
    if ((millis() - motor_timer) > motor_duration) //motor_duration
    {  
      motor_timer = millis();
      right_motor::stop();
      Serial.println("Stopped R Motor");   
      //clear encoder  
      resetEncoderR();
      if (eCountR != 0) eCountR = 0;
      delay(3000);  
    }

    /*
    // Checking voltage sent to motors
    units::percentage twenty(20.0);
    units::percentage forty(40.0);
    units::percentage sixty(60.0);
    units::percentage eighty(80.0);

    right_motor::forward(twenty);
    delay(2000);
    right_motor::stop();
    delay(2000);
    right_motor::forward(forty);
    delay(2000);
    right_motor::stop();
    delay(2000);
    right_motor::forward(sixty);
    delay(2000);
    right_motor::stop();
    delay(2000);
    right_motor::forward(eighty);
    delay(2000);
    right_motor::stop();
    delay(10000);
    */
  #endif
   
  //3. Test Motor 2 Functionality
  
   // 3. Run motor 2 at half speed in the backward direction for 5 seconds (2), 
   // display the readings of the associated encoder on the serial monitor of Arduino (2), 
   // and display the distance travelled by the wheel

  #ifdef MOTOR_2_TEST
    
    left_motor::backward(mHalfSpeed); //half speed = 50.0

    //Print encoder count + distance travelled by wheel
    Serial.print("lEncoder: "); Serial.print(eCountL);

    // OOP Version
    //units::millimeters lDistance = left_wheel::position();    
    //Serial.print(" | lWheel:dist: "); Serial.println(lDistance.count());  

    Serial.print(" | lWheel(mm): "); Serial.println(abs((eCountL/COUNTS_PER_REV)*WHEEL_CIRCUM)); 

    if ((millis() - motor_timer) > motor_duration)
    {  
      motor_timer = millis();
      left_motor::stop();
      Serial.println("Left Motor Stopped");  
      resetEncoderL();
      if (eCountL != 0) eCountL = 0;
      delay(3000);
    }
       
  #endif

  //4. Read Left Encoder

  /*
   * 4. Control the left wheel to run for an angle (X deg) given by the demonstrator before the test and display 
   * the readings of the associated encoder on the serial monitor. The student should define a variable 
   * in their program to store this value and allow the demonstrator to change the value of this 
   * variable before the assessment. (Positive value means the wheel should move forward 
   * if mounted on the left side of the robot) (3)
   */

  #ifdef L_ENCODER_TEST

    int angle = 3 * LEFT_WHEEL_ANGLE; // aiming for +/- 15 degrees
    //~ 1 encoder tick = 0.5 degrees
    
    if (angle > 0) left_motor::forward(mtestSpeed); 
    else left_motor::backward(mtestSpeed);
    
    Serial.print("lEncoder: "); Serial.println(eCountL);

    // Calculation of "error" given the setpoint
    if ( ((abs(angle) - abs((eCountL * 0.5))) <= ANGLE_OFFSET) || abs((eCountL * 0.5)) >= (abs(angle) + ANGLE_OFFSET))
    {
       left_motor::stop();
       resetEncoderL();
       if (eCountL != 0) eCountL = 0;
       delay(1000);
    } 

  #endif

  //5. Read RightWheel Encoder

  /*
   * 5. Control the right wheel to run for an angle (Y deg) given by the demonstrator before the test and display 
   * the readings of the associated encoder on the serial monitor. The student should
   * define a variable in their program to store this value and allow the demonstrator to change the value
   * of the variable before the assessment. (Positive value means the wheel should move forward 
   * if mounted on the right side of the robot) (3)
   */

  #ifdef R_ENCODER_TEST

    int angle = RIGHT_WHEEL_ANGLE;
    //~ 1 encoder tick = 0.5 degrees
    
    if (angle > 0)right_motor::forward(mtestSpeed); 
    else right_motor::backward(mtestSpeed);
    
    Serial.print("REncoder: "); Serial.println(-eCountR);

    // Calculation of "error" given the setpoint
    if ( ((abs(angle) - abs((eCountR * 0.5))) <= ANGLE_OFFSET) || abs((eCountR * 0.5)) >= (abs(angle) + ANGLE_OFFSET))
    {
       right_motor::stop();
       resetEncoderR();
       delay(1000);
    } 


  #endif

  delay(50);

}
