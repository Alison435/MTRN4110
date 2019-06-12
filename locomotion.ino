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

//#define USE_BLINK
#define USE_FADE
//#define USE_ATEST //TODO!
//#define MOTOR_TEST //TODO!
//#define ENCODER_TEST //TODO!

void setupDigital()
{
  led::config_io_mode(io_mode::output);
  statusRed::config_io_mode(io_mode::output);
  statusGreen::config_io_mode(io_mode::output);
}

void setupMotor()
{  
  in1::config_io_mode(io_mode::output); 
  in2::config_io_mode(io_mode::output); 
  in3::config_io_mode(io_mode::output); 
  in4::config_io_mode(io_mode::output); 
  //right_motor::enable();
  //left_motor::enable();
}

void setup() {

  Serial.begin(115200);
  setupDigital();
  setupMotor(); 
}

void loop() {

  //1. Digital I/O Functionality

  #ifdef USE_BLINK
    //Blink test using API
    statusRed::high();
    statusGreen::low();      
    delay(500);
    
    statusRed::low();
    statusGreen::high();      
    delay(500);
  #endif
  
  #ifdef USE_FADE
    //Fade test using API on Green LED @ PWM pin #3
    statusGreen::pwm_write(); //argument of form units::percentage duty_cycle
    delay(50);
    statusGreen::pwm_write(50.0);
    delay(50);
    statusGreen::pwm_write(75.0);
    delay(50);
    statusGreen::pwm_write(100.0);
    delay(50);
  #endif
  
  //2. Analog I/O Functionality

  #ifdef ATEST
    double analogVal = analogPin::analog_read();
    Serial.println(analogVal);
  #endif
  
  //3. Motor Control Functionality

  #ifdef MOTOR_TEST
    right_motor::forward()
    delay(100);
    right_motor::backward();
    delay(100);
  #endif

  //4. Read Wheel Encoder + Distance Travelled

  #ifdef ENCODER_TEST
    //test encoder count/direction
    //encoder::
  #endif


}
