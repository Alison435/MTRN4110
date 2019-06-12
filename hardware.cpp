#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"

using pin_t = uint8_t; //uint8_t is a byte

// Have 'hardware::digital_pin::'' as digital_pin is the template class which I am defining
// the functions for, which is in the namespace of hardware.
// In Arduino sketch, added 'using namespace hardware' to avoid repetitive scope operator usage

// 1) Implementation of digital_pin Class

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

template <pin_t pin> //works
static auto hardware::digital_pin<pin>::read () -> logic_level
{

    logic_level readLevel;

    if (digitalRead(pin) == 1)
    {
        // HIGH was read at digital pin
        readLevel = logic_level::high;
        //Serial.println("read pin as HIGH");
    }

    if (digitalRead(pin) == 0)
    {
        // LOW was read at digital pin
        readLevel = logic_level::low;
        //Serial.println("read pin as LOW");
    }

    return readLevel;

}

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

template <pin_t pin> //works
static auto hardware::digital_pin<pin>::high() -> void
{

    digitalWrite(pin,HIGH);
    Serial.println("Pin is HIGH");

}

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



    //value: the duty cycle: between 0 (always off) and 255 (always on). Allowed data types: int.
    analogWrite(pin,duty_cycle);

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

// 2) Implementation of analog_pin Class

    /**
     * \brief The set_analog_reference method set the reference voltage for
     * analog read.
     * \param ref is the analog reference.
     */
template <typename base>
static auto hardware::analog_pin<base>::set_analog_reference (analog_reference ref) -> void
{
     switch (ref)
    {

        case (analog_reference::vcc_default):
        {
            analogReference(DEFAULT);
            break;
        }

        case (analog_reference::internal_1v1):
        {
            analogReference(INTERNAL1V1);
            break;
        }

        case (analog_reference::internal):
        {
            analogReference(INTERNAL);
            break;
        }

        case (analog_reference::internal_2v56):
        {
            analogReference(INTERNAL2V56);
            break;
        }

        case (analog_reference::external):
        {
            analogReference(EXTERNAL);
            break;
        }

        default:
            break;

    }
}

    /**
     * \brief The analog_read method analog input.
     * \return voltage in volts.
     */
template <typename base>
static auto hardware::analog_pin<base>::analog_read () -> units::volts
{

    units::volts analogVoltage;
    analog_reference aRef;

    using analog_io = hardware::analog_pin<base>;

    analogVoltage = (analogRead(pin) * aRef) / analog_io::conversion_unit;

    return analogVoltage;

}

// 3) Implementation of motor Class

/**
 * \brief The motor represents one motor channel of the motor driver. See motor
 * driver data sheets for digital output combination to control the direction of
 * motor. \tparam pin_a is direction pin a \tparam pin_b is direction pin b
 */

/*
enum class drive_direction : uint8_t
{
    unknown,
    forward,
    backward
};
*/

    /**
     * \brief The enable method enables motor control pins.
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::enable () -> void
{


}

    /**
     * \brief The stop method stops the motor
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::stop () -> void
{


}

    /**
     * \brief The forward method makes the motor goes forward.
     * \param velocity is percentage of maximum speed of motor.
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::forward (units::percentage velocity) -> void
{


}

    /**
     * \brief The forward method makes the motor goes backward.
     * \param velocity is percentage of maximum speed of motor.
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::backward (units::percentage velocity) -> void
{


}

// ---------------TODO: Jono-------------------

// 4) Implementation of encoder Class {awaiting physical parts for testing but will start code soon}
// 5) Implementation of wheel Class

// ---------------TODO: Jono-------------------

// NOTE: Explicit Instantiation of Template Classes...

// Digital/Analog I/O

template class hardware::digital_pin<13U>;
template class hardware::digital_pin<8U>;
template class hardware::digital_pin<6U>; //LED fade test pin
template class hardware::analog_pin<hardware::digital_pin<1U>>; //for analog test

// Motor Driver (H-bridge)

template class hardware::motor<hardware::pins::in1, hardware::pins::in2>; //Pins 2,4
template class hardware::motor<hardware::pins::in3, hardware::pins::in4>;  //Pins 9,10

// Encoder

