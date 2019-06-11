// MTRN4110 Phase A : Locomotion Hardware/Software Interfacing
// Created by Jonathan Loong (z5014751)

// Including Libraries for Phase A
#include <units.h>
#include <hardware.h>
#include <type_traits.h>
#include <hardware_definition.h>

// Subsequent code uses the low-level namespace of hardware
// Avoids prepending scope operator to all functions
using namespace hardware;

//Note: The .cpp file has explicit instantiations
// -Instantiating objects for each desired class-

// Tests for Phase A demonstrations
#define USE_BLINK
//#define USE_FADE
//#define USE_ATEST //TODO!
//#define MOTOR_TEST //TODO!
//#define ENCODER_TEST //TODO!

// i) objects for pin I/O
led onboardLed;
statusRed ledR;
statusGreen ledG; //used for fade pwm_write test
analogTest aTest; 

//(error with pins:: having undefined reference)
// ii) objects for motors 
//right_motor rMotor;
pins::en1 speedR; //enable pin for motor (h-bridge)
pins::in1 dir1;
pins::in2 dir2;

// iii) units
//units::microseconds microCheck; 
//units::milliseconds millisCheck;

void setup() {

  Serial.begin(115200);
  
  // a) set up for digital I/O pins (make into function)
  onboardLed.config_io_mode(io_mode::output);
  ledR.config_io_mode(io_mode::output);
  ledG.config_io_mode(io_mode::output);

  // b) set up for motor: (make into function)  
  
  //pwm control for right motor terminal
  //speedR.config_io_mode(io_mode::output);
  //rMotor.enable()
     
  //dir1 and dir2 on right motor terminal
  //dir1.config_io_mode(io_mode::output); 
  //dir2.config_io_mode(io_mode::output); 
  
}

void loop() {

  //1. Digital I/O Functionality

  #ifdef USE_BLINK
    //Blink test using API
    ledG.high();
    ledR.low();      
    //microCheck.now();
    delay(1000);
    
    ledG.low();
    ledR.high();      
    //millisCheck.now();
    delay(1000);
  #endif
  
  #ifdef USE_FADE
    //Fade test using API on Green LED @ PWM pin #3
    ledG.pwm_write(25.0);
    delay(50);
    ledG.pwm_write(50.0);
    delay(50);
    ledG.pwm_write(75.0);
    delay(50);
    ledG.pwm_write(100.0);
    delay(50);
  #endif
  
  //2. Analog I/O Functionality

  #ifdef ATEST
    double analogVal = aTest.analog_read();
    Serial.println(analogVal);
  #endif
  
  //3. Motor Control Functionality

  #ifdef MOTOR_TEST
    rMotor.forward();
    delay(100);
    rMotor.backward();
    delay(100);
    //speedR.pwm_write(50.0); //50% speed
  #endif

  //4. Read Wheel Encoder + Distance Travelled

  #ifdef ENCODER_TEST
    //test encoder count/direction
    
  #endif


}
