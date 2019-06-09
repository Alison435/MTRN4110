// MTRN4110 Phase A : Locomotion Hardware/Software Interfacing
// Created by Jonathan Loong (z5014751)

// Including Libraries for Phase A
#include <units.h>
#include <hardware.h>
#include <type_traits.h>
#include <hardware_definition.h>

// Subsequenct code uses the low-level namespace of hardware
// Avoids prepending scope operator to all functions
using namespace hardware;

//Note: The .cpp file has explicit instantiations
// -----------Instantiating objects for each desired class HERE------

led onboardLed;
statusRed ledR;
statusGreen ledG;

void setup() {

  Serial.begin(115200);
  
  //replaced pinMode(pin_number,OUTPUT) with API version;
  onboardLed.config_io_mode(io_mode::output);
  ledR.config_io_mode(io_mode::output);
  ledG.config_io_mode(io_mode::output);
    
}

void loop() {

  //1. Digital I/O Functionality

  //Blink test using API
  ledG.high();
  ledR.low();
  logic_level val = ledG.read();
  Serial.println(int(val));
  delay(1000);
  
  ledG.low();
  ledR.high(); 
  logic_level val2 = ledG.read();
  Serial.println(int(val2)); 
  delay(1000);
  
  //2. Analog I/O Functionality
  
  //3. Motor Control Functionality
  
  //4. Read Wheel Encoder
  
  //5. Distance Travelled


}
