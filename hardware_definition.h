#pragma once
// __STUDENT_SHOULD_CHANGE_THIS_FILE__
// Example of how hardware definition could look like.

namespace hardware
{
/**
 * \brief The pins namespace contains definition of the functionality of all the
 * pins on Arduino.
 */
    namespace pins    // TODO student to define.
    {

        // 1) Pins for Sensors

        // imu_interrupt is used by imu to signal data ready.
        using imu_interrupt = digital_pin<1U>;

        // The sonar_trigger is ultrasound trigger pin connected to a0.
        using sonar_trigger = analog_pin<digital_pin<6U>>;

        // The ultrasound_echo is ultrasound echo pin connected to a1.
        using sonar_echo = analog_pin<digital_pin<7U>>;

        using left_lidar_enable = digital_pin<12U>;
        using right_lidar_enable = digital_pin<13U>;

        constexpr auto left_lidar_address = 0x22;
        constexpr auto right_lidar_address = 0x42;

        // 2) Pins for Motor/Encoder

       // Right Motor (Direction and Speed)
        using in1 = digital_pin<2U>;
        using in2 = digital_pin<4U>;
        using en1 = digital_pin<3U>;

        // Left Motor (Direction and Speed)
        using in3 = digital_pin<9U>;
        using in4 = digital_pin<10U>;
        using en2 = digital_pin<12U>;

        using left_encoder_a = interrupt<digital_pin<8U>>;
        using left_encoder_b = digital_pin<9U>;
        using right_encoder_a = interrupt<digital_pin<10U>>;
        using right_encoder_b = digital_pin<11U>;

    }    // namespace pins

// 1. Pins for I/O Hardware

// Onboard LED (debugging)
using led = digital_pin<13U>;

// Red LED
using statusRed = digital_pin<8U>;

// Green LED
using statusGreen = digital_pin<9U>;

// 2. Pins for Encoder, Motor and Wheel

using left_wheel = wheel<pins::left_encoder_a, pins::left_encoder_b>;
using right_wheel = wheel<pins::right_encoder_a, pins::right_encoder_b>;

using left_encoder = encoder<pins::left_encoder_a, pins::left_encoder_b>;
using right_encoder = encoder<pins::right_encoder_a, pins::right_encoder_b>;

using right_motor = motor<pins::in1, pins::in2>;
using left_motor = motor<pins::in3, pins::in4>;

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

}    // namespace hardware
