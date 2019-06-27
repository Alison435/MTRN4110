#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"

using pin_t = uint8_t; //uint8_t is a byte

#define PI_ 3.14
#define WHEEL_CIRCUM_ 240.0 //in mm
#define COUNTS_PER_REV_ 16.0

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
    //Serial.println("Pin is HIGH");

}

template <pin_t pin> //works
static auto hardware::digital_pin<pin>::low() -> void
{

    digitalWrite(pin,LOW);
    //Serial.println("Pin is LOW");

}

template <pin_t pin>
static auto hardware::digital_pin<pin>::pwm_write (units::percentage duty_cycle) -> void
{

    //overloaded * operator. allow for unit_type / double multiplication
    analogWrite(pin,duty_cycle.count() * (1/100.0) * 255);
}

template <pin_t pin>
static auto hardware::digital_pin<pin>::pulse_length (logic_level state = logic_level::high,
units::microseconds timeout = 1000000_us) -> units::microseconds
{

    //TODO!!

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

        //case (analog_reference::internal): //only on UNO
        //{
        //    analogReference(INTERNAL);
        //    break;
        //}

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

template <typename base>
static auto hardware::analog_pin<base>::analog_read () -> units::volts
{

    //units::volts analogVoltage(0.0);
    //analogVoltage.count() = analogRead(analog_pin<base>) * (5.0/1024);

    //return analogVoltage.count();

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

    units::percentage stopPWM(0.0);

    pin_a::pwm_write(stopPWM);
    pin_b::pwm_write(stopPWM);

}

template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::forward (units::percentage velocity) -> void
{

    //send one motor to 1 and one to 0
    pin_a::write(logic_level::high);
    pin_b::write(logic_level::low);

    if (pin_a::read() == logic_level::high && pins::in1::read() == logic_level::high)
    {

        hardware::en1::pwm_write(velocity);
        //Serial.println("Right M Forward");

    }
    else //other motor is desired
    {

        hardware::en2::pwm_write(velocity);
        //Serial.println("Left M Forward");
    }
}

template <class pin_a, class pin_b>
static auto hardware::motor<pin_a,pin_b>::backward (units::percentage velocity) -> void
{

    //send one motor to 0 and one to 1
    pin_a::write(logic_level::low);
    pin_b::write(logic_level::high);

    if (pin_b::read() == logic_level::high && pins::in2::read() == logic_level::high)
    {

        hardware::en1::pwm_write(velocity);
        //Serial.println("Right M Backward");
    }
    else //other motor is desired
    {
        hardware::en2::pwm_write(velocity);
        //Serial.println("Left M Backward");
    }

}

// ------------------- 4) Implementation of encoder Class -------------------

// Encoder variable for ISR using OOP
using encoder_count = int;
volatile byte aCount;
volatile encoder_count eCounter; // Encoder count used for OOP
boolean Direction = true; // Rotation direction

// For each encoder:
// Pin a = interrupt
// Pin b = digital pin

template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::enable () -> void
{

    // enable encoder pins
    pin_a::config_io_mode (io_mode::input);
    pin_b::config_io_mode (io_mode::input);
    Direction = true;

}

    /**
     * \brief The count method returns the current count of the encoder.
     * \return current count.
     */

// This function was essentially a conversion from pure Arduino to OOP
// However, this function is believed to be the ISR that is called whenever
// the encoder pin is interrupted, should ideally be a void return value.
template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::count () -> encoder_count
{

    //alias of encoder_count = int;
    // Get count from interrupt of A and B channels?

    encoder_count Lstate = encoder_count(pin_a::read());

    //Serial.print("A: "); Serial.println(Lstate);

    if((aCount == 0) && Lstate == 1)
    {
        encoder_count encoder_b = encoder_count(pin_b::read());

        //Serial.print("B: "); Serial.println(encoder_b);
        //Serial.println("");

        if(encoder_b == 0 && Direction)
        {
          Direction = false; //Reverse

        }
        else if(encoder_b == 1 && !Direction)
        {
          Direction = true;  //Forward
        }
    }

    aCount = Lstate;

    if(!Direction)  eCounter++;
    else  eCounter--;

    return eCounter;

}

    /**
     * \brief The reset method reset the current count and position of the
     * encoder.
     */
template <typename pin_a, typename pin_b>
static auto hardware::encoder<pin_a,pin_b>::reset () -> void
{

    eCounter = 0;

}

// ------------------- 5) Implementation of wheel Class -------------------

     //brief The position method returns the distance traveled by the wheel,
     //assume no slip. \return the distance traveled by the wheel
template <typename pin_a, typename pin_b>
static auto hardware::wheel<pin_a,pin_b>::position () -> units::millimeters
{

   units::millimeters wheelDistance(1.0);

   // distnace =  (wheel circumference)*counts/(counts per complete revolution)
   (encoder<pin_a, pin_b>::count()/ COUNTS_PER_REV_) * WHEEL_CIRCUM_;

   return wheelDistance;

}

/*
// ---------------- Implementation of interrupt class --------------

//eg: using left_encoder_a = interrupt<digital_pin<20U>>;

template <typename pin>
static auto hardware::interrupt<pin>::attach_interrupt (void (*callback) (),
    interrupt_mode mode = interrupt_mode::rising) -> void
{

    // callback function = function another function calls when a condition is met
    //attachInterrupt(pin,(*callback)(), interrupt_mode::rising);

}

template <typename pin>
static auto hardware::interrupt<pin>::detach_interrupt () -> void
{

    //detachInterrupt(digitalPinToInterrupt(pin));

}

*/

// NOTE: Explicit Instantiation of Template Classes...

// Digital/Analog I/O

template class hardware::digital_pin<13U>;
template class hardware::digital_pin<5U>;
template class hardware::digital_pin<6U>; //LED fade test pin
template class hardware::analog_pin<hardware::digital_pin<1U>>;

// Motor Driver (H-bridge)

template class hardware::digital_pin<10U>;
template class hardware::digital_pin<11U>;
template class hardware::motor<hardware::pins::in1, hardware::pins::in2>; //Pins 2,4
template class hardware::motor<hardware::pins::in3, hardware::pins::in4>;  //Pins 9,10

// Encoder and Wheel

template class hardware::wheel<hardware::pins::left_encoder_a, hardware::pins::left_encoder_b>;
template class hardware::wheel<hardware::pins::right_encoder_a, hardware::pins::right_encoder_b>;
template class hardware::encoder<hardware::pins::left_encoder_a, hardware::pins::left_encoder_b>;
template class hardware::encoder<hardware::pins::right_encoder_a, hardware::pins::right_encoder_b>;