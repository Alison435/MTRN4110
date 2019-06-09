#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"

using pin_t = uint8_t; //uint8_t is a byte. (Arduino - mentions need for const unsigned char)

// Have 'hardware::digital_pin::'' as digital_pin is the template class which I am defining
// the functions for, which is in the namespace of hardware.
// In Arduino sketch, added 'using namespace hardware' to avoid repetitive scope operator usage

// 1) Implementation of digital_pin Class

    /**
     * \brief The config_io_mode method set I/O mode to the digital pin.
     * Should be called to set the correct mode before any I/O activity.
     * \param mode is the desired I/O mode.
     */

template <pin_t pin> //works
static auto hardware::digital_pin<pin>::config_io_mode (io_mode mode) -> void
{

    switch (mode)
    {

        case (io_mode::input):
        {
            pinMode(pin,INPUT);
            break;
        }

        case (io_mode::output):
        {
            pinMode(pin,OUTPUT);
            break;
        }

        case (io_mode::input_pullup):
        {
            pinMode(pin,INPUT_PULLUP);
            break;
        }

        case (io_mode::unset):
        {
            Serial.println("Pin is not set yet");
            break;
        }

        default:
            break;

    }
}

    /**
    * \brief The read method returns the current pin input.
    * \return the input logic level.
    */
template <pin_t pin>
static auto hardware::digital_pin<pin>::read () -> logic_level
{

    logic_level readLevel;

    if (digitalRead(pin) == 1)
    {
        // HIGH was read at digital pin
        readLevel = logic_level::high;
        Serial.println("read pin as HIGH");
    }

    if (digitalRead(pin) == 0)
    {
        // LOW was read at digital pin
        readLevel = logic_level::low;
        Serial.println("read pin as LOW");
    }

    return readLevel;

}

    /**
    * \brief The write method set digital pin output to mode.
    * \param level is the logic level to set the digital pin to.
    */
template <pin_t pin>
static auto hardware::digital_pin<pin>::write (logic_level level) -> void
{

    if (level == logic_level::high)
    {
        digitalWrite(pin,HIGH); //set pinMode to HIGH using Arduino syntax
    }
    if (level == logic_level::low)
    {
        digitalWrite(pin,LOW);
    }

}

    /**
    * \brief The high method set digital output level to high.
    */
template <pin_t pin> //works
static auto hardware::digital_pin<pin>::high() -> void
{

    digitalWrite(pin,HIGH);
    Serial.println("Pin is HIGH");

}

    /**
    * \brief The low method set digital output level to low.
    */
template <pin_t pin> //works
static auto hardware::digital_pin<pin>::low() -> void
{

    digitalWrite(pin,LOW);
    Serial.println("Pin is LOW");

}

    //brief The pwm_write method writes an analog voltage out as PWM wave.
    //This function does not check if the pin is capable of PWM.
    // Uno pin 3, 5, 6, 9, 10, 11. Mega: 2 - 13 and 44 - 46.
    // PWM at 490Hz except pin 5 and 6 are 980Hz.
    //param duty_cycle value between 0.0 and 100.0 representing is the PWM
    // duty cycle of 0% to 100%.

template <pin_t pin>
static auto hardware::digital_pin<pin>::pwm_write (units::percentage duty_cycle) -> void
{



}

    //
    // \brief pulse_length measures the duration of a pulse in microseconds.
    // With either rising or falling edge as trigger.
    // \param state HIGH for rising edge trigger, LOW for falling edge trigger
    // start.
    // \param timeout in microseconds
    // \return pulse length in
    // microseconds.

template <pin_t pin>
static auto hardware::digital_pin<pin>::pulse_length (logic_level state = logic_level::high,
units::microseconds timeout = 1000000_us) -> units::microseconds
{



}

// ---------------TODO: Jono-------------------

// 2) Implementation of analog_pin Class
// 3) Implementation of motor Class
// 4) Implementation of encoder Class
// 5) Implementation of wheel Class

// ---------------TODO: Jono-------------------

// NOTE: Explicit Instantiation of Template Classes...
// Template was split into .cpp and .h file. Compiler now knows that it will
// compile the .cpp for the classes using the specified pins

template class hardware::digital_pin<13U>; //need to ask why using led = digital_pin<13U> doesn't work here!
template class hardware::digital_pin<8U>;
template class hardware::digital_pin<7U>;
