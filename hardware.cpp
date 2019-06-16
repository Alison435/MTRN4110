#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"

#define PI 3.14
#define WHEEL_CIRCUM 240 //in mm

using pin_t = uint8_t; //uint8_t is a byte

//#define USING_MEGA

// ------------------- 1) Implementation of digital_pin Class -------------------

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

template <pin_t pin>
static auto hardware::digital_pin<pin>::pwm_write (units::percentage duty_cycle) -> void
{

    //overloaded / operator. allow for unit_type / double multiplication
    //analogWrite(pin,(duty_cycle * (1/100.0) * 255));
}

template <pin_t pin>
static auto hardware::digital_pin<pin>::pulse_length (logic_level state = logic_level::high,
units::microseconds timeout = 1000000_us) -> units::microseconds
{
    //using Arduino pulseIn()
}

// ------------------- 2) Implementation of analog_pin Class -------------------

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

        case (analog_reference::internal): //only on UNO
        {
            analogReference(INTERNAL);
            break;
        }

        case (analog_reference::external):
        {
            analogReference(EXTERNAL);
            break;
        }

        #ifdef USING_MEGA

            case (analog_reference::internal_2v56): //only on MEGA
            {
                analogReference(INTERNAL2V56);
                break;
            }

            case (analog_reference::internal_1v1): //only on MEGA
            {
                analogReference(INTERNAL1V1);
                break;
            }

        #endif

        default:
            break;

    }
}

// ------------------- 3) Implementation of motor Class -------------------

    /**
     * \brief The enable method enables motor control pins.
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::enable () -> void
{

    // Direction control pins on H-bridge (set to output mode)
    pin_a::config_io_mode(io_mode::output);
    pin_b::config_io_mode(io_mode::output);

}

    /**
     * \brief The stop method stops the motor
     */
template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::stop () -> void
{

    //send 0 to both motor pins
    pin_a::write(logic_level::low);
    pin_b::write(logic_level::low);

}

//class percentage : public base_unit<percentage, double>
// For base_unit class -> template <typename derived_units_name, typename value_t = double>

template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::forward (units::percentage velocity) -> void
{

    drive_direction driveD = drive_direction::forward;

    //send one motor to 1 and one to 0
    pin_a::write(logic_level::high);
    pin_b::write(logic_level::low);

    // Logic to determine if motor 1 or motor 2
    // match enable pin with designated motor?

    /*

    if ()
    {

    }

    */

    //send desired PWM
    //Serial.println(velocity.count());
    hardware::en1::pwm_write(velocity);

}

template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::backward (units::percentage velocity) -> void
{

    drive_direction driveD = drive_direction::backward;

    //send one motor to 0 and one to 1
    pin_a::write(logic_level::low);
    pin_b::write(logic_level::high);

    //send desired PWM
    hardware::en1::pwm_write(velocity); //make dynamic somehow...to suit different motor enable pins

}

// ---------------- Implementation of interrupt class --------------

//eg: using left_encoder_a = interrupt<digital_pin<20U>>;

    //brief The attach_interrupt method set the interrupt service routine
    //(ISR) that will be called when an interrupt is triggered.
    //\param callback
    //function with function signature of void meow ();
    //param mode is the
    //interrupt trigger condition.
template <typename pin>
static auto hardware::interrupt<pin>::attach_interrupt (void (*callback) (),interrupt_mode mode = interrupt_mode::rising) -> void
{

    // callback function = function another function calls when a condition is met
    attachInterrupt(pin,(*callback)(), mode);

}

template <typename pin>
static auto hardware::interrupt<pin>::detach_interrupt () -> void
{

    detachInterrupt(digitalPinToInterrupt(pin));

}

// ------------------- 4) Implementation of encoder Class -------------------

// Pin a = interrupt
// Pin b = digital pin

template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::enable () -> void
{

    // enable encoder pins
    pin_a::config_io_mode (io_mode::output);
    pin_b::config_io_mode (io_mode::output);

}

    /**
     * \brief The count method returns the current count of the encoder.
     * \return current count.
     */

template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::count () -> encoder_count
{

    //alias of encoder_count = int;
    // Get count from interrupt of A and B channels


}

    /**
     * \brief The reset method reset the current count and position of the
     * encoder.
     */
template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::reset () -> void
{



}

// ------------------- 5) Implementation of wheel Class -------------------

     //brief The position method returns the distance traveled by the wheel,
     //assume no slip. \return the distance traveled by the wheel
template <typename pin_a, typename pin_b>
static auto hardware::wheel<pin_a,pin_b>::position () -> units::millimeters
{

    //PI
    //WHEEL_CIRCUM

    //Distance Travelled = (Encoder Count / 360) * WHEEL_CIRCUM

}


// NOTE: Explicit Instantiation of Template Classes...

// Digital/Analog I/O

template class hardware::digital_pin<13U>;
template class hardware::digital_pin<8U>;
template class hardware::digital_pin<6U>; //LED fade test pin
template class hardware::analog_pin<hardware::digital_pin<1U>>; //for analog test

// Motor Driver (H-bridge)

template class hardware::digital_pin<3U>;
template class hardware::digital_pin<12U>;
template class hardware::motor<hardware::pins::in1, hardware::pins::in2>; //Pins 2,4
template class hardware::motor<hardware::pins::in3, hardware::pins::in4>;  //Pins 9,10

// Encoder and Wheel

template class hardware::wheel<hardware::pins::left_encoder_a, hardware::pins::left_encoder_b>;
template class hardware::wheel<hardware::pins::right_encoder_a, hardware::pins::right_encoder_b>;
template class hardware::encoder<hardware::pins::left_encoder_a, hardware::pins::left_encoder_b>;
template class hardware::encoder<hardware::pins::right_encoder_a, hardware::pins::right_encoder_b>;
