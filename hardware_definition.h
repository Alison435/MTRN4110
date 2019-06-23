#pragma once
// __STUDENT_SHOULD_CHANGE_THIS_FILE__
// Example of how hardware definition could look like.

namespace hardware
{
/**
 * \brief The pins namespace contains definition of the functionality of all the
 * pins on Arduino.
 */
    namespace pins
    {

        // 1) Pins for Sensors

        // imu_interrupt is used by imu to signal data ready.
        using imu_interrupt = digital_pin<18U>;

        // The sonar_trigger is ultrasound trigger pin
        using sonar_trigger = digital_pin<6U>; //<digital_pin<12U>

        // The ultrasound_echo is ultrasound echo pin
        using sonar_echo = digital_pin<7U>; //<digital_pin<13U>;

        using left_lidar_enable = digital_pin<12U>; //<digital_pin<20U>; //SDA on Mega
        using right_lidar_enable = digital_pin<13U>; //<digital_pin<21U>; //SCL on Mega

        constexpr auto left_lidar_address = 0x22; //TODO: check i2C addresses
        constexpr auto right_lidar_address = 0x42;
        constexpr auto imu_lidar_address = 0x68;

        // 2) Pins for Motor/Encoder

        // Right Motor (Direction)
        using in1 = digital_pin<7U>; //<digital_pin<8U>;
        using in2 = digital_pin<8U>; //<digital_pin<9U>;

        // Left Motor (Direction)
        using in3 = digital_pin<11U>; //<digital_pin<14U>;
        using in4 = digital_pin<12U>; //<digital_pin<15U>;

        // Encoder interrupt pins
        using left_encoder_a = digital_pin<2U>; //<digital_pin<2U>;
        using left_encoder_b = digital_pin<1U>; //<digital_pin<22U>;

        using right_encoder_a = digital_pin<3U>; //digital_pin<3U>;
        using right_encoder_b = digital_pin<4U>; //<digital_pin<24U>;



    }    // namespace pins

// 1. Pins for I/O Hardware

// Onboard LED (debugging)
using led = digital_pin<13U>;

// Red LED
using statusRed = digital_pin<8U>; //<digital_pin<2U>;

// Green LED
using statusGreen = digital_pin<6U>; //<digital_pin<3U>;

// 2. Pins for Encoder, Motor and Wheel

using right_motor = motor<pins::in1, pins::in2>;
using left_motor = motor<pins::in3, pins::in4>;
using en1 = digital_pin<9U>; //used for motor pwm //<digital_pin<10U>;
using en2 = digital_pin<10U>; //used for motor pwm //<digital_pin<11U>;

using left_wheel = wheel<pins::left_encoder_a, pins::left_encoder_b>;
using right_wheel = wheel<pins::right_encoder_a, pins::right_encoder_b>;

using left_encoder = encoder<pins::left_encoder_a, pins::left_encoder_b>;
using right_encoder = encoder<pins::right_encoder_a, pins::right_encoder_b>;

// 3. Pins for Sensors

// Ultrasonic sensor template bind to the trigger and echo pin.
using front_sonar = sonar<pins::sonar_trigger, pins::sonar_echo>;
using left_lidar = lidar<lidar_tag<0>>;
using right_lidar = lidar<lidar_tag<1>>;

// 4. Pins for Serial Communication

// This serial is used to communicate with pc via usb.
using serial = serial_api<serial_tag<0>>;

// The serial is used to communicate with Bluetooth module.
using bluetooth = serial_api<serial_tag<1>>;

// Pins for LCD

}    // namespace hardware
