// MTRN4110 Phase A : Locomotion Hardware/Software Interfacing Test
// Created by Jonathan Loong (z5014751) 

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

#define LEFT_WHEEL_ANGLE 45
#define RIGHT_WHEEL_ANGLE 45

//#define TOGGLE_LED
#define MOTOR_1_TEST
//#define MOTOR_2_TEST
//#define L_ENCODER_TEST
//#define R_ENCODER_TEST

void setupDigitalPins()
{
  led::config_io_mode(io_mode::output);
  statusRed::config_io_mode(io_mode::output);
  statusGreen::config_io_mode(io_mode::output);
}

void setupMotor()
{  
  en1::config_io_mode(io_mode::output); //for pwm control of right motor
  en2::config_io_mode(io_mode::output); //for pwm control of left motor
  right_motor::enable();
  left_motor::enable();
}

void setupEncoderWheel()
{
 
  left_wheel::enable();
  right_wheel::enable();
  left_encoder::enable();
  right_encoder::enable();

}

// Timing for hardware tests
unsigned long motor_duration = 5000;
unsigned long motor_timer = 0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;  // LED logic level (to be toggled)

// Units for Encoder + Motor + Wheel (using C++ API)
units::percentage mFullSpeed(25.0);
units::percentage mHalfSpeed(50.0);


void setup() {

  Serial.begin(115200);
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel();
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

    case (TEST_2):
    
      if (millis() - motor_timer > motor_duration)
      {
        motor_timer = millis();    
        right_motor::stop();  
        delay(15);
        system_mode = CHOOSE_TEST;
      }
    
      right_motor::forward();   
      
      break;

    case (TEST_3):
    
      if (millis() - motor_timer > motor_duration)
      {
        motor_timer = millis();    
        left_motor::stop();  
        delay(15);
        system_mode = CHOOSE_TEST;
      }
    
      left_motor::backward();   
      
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

  /*
   * 2. Run motor 1 at full speed in the forward direction for 5 seconds (2), 
   * display the readings of the associated encoder on the serial monitor of Arduino (2), 
   * and display the distance travelled by the wheel
   */
  
  #ifdef MOTOR_1_TEST

    right_motor::forward(mFullSpeed); //full speed = 100.0; (25.0 atm)
    delay(1000);
    right_motor::backward(mFullSpeed);
    delay(1000);
    right_motor::stop();

    /*
    if (millis() - motor_timer > motor_duration)
    {
      motor_timer = millis();    
      right_motor::stop();  
    }
    
    right_motor::forward(mFullSpeed);          
    */
    
  #endif
   
  //3. Test Motor 2 Functionality

  /*
   * 2. Run motor 2 at half speed in the backward direction for 5 seconds (2), 
   * display the readings of the associated encoder on the serial monitor of Arduino (2), 
   * and display the distance travelled by the wheel
   */

  #ifdef MOTOR_2_TEST

    right_motor::backward(mHalfSpeed); //half speed = 50.0
    delay(100);
    right_motor::forward(mHalfSpeed);
    delay(100);
    right_motor::stop();

    /*
    if (millis() - motor_timer > motor_duration)
    {
      motor_timer = millis();    
      left_motor::stop();  
    }
    
    left_motor::backward(mHalfSpeed);          
    */
    
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
  
    l_encoder::count();
    
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

  

  #endif

  delay(15);

}
