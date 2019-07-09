 namespace hardware
    {
        namespace pins
        {

            // Right Motor (Direction)
            using in1 = digital_pin<12U>;
            using in2 = digital_pin<13U>;

            // Left Motor (Direction
            using in3 = digital_pin<8U>;
            using in4 = digital_pin<9U>;

            // Encoder interrupt pins
            using left_encoder_a = digital_pin<2U>;
            using left_encoder_b = digital_pin<1U>;

            using right_encoder_a = digital_pin<3U>;
            using right_encoder_b = digital_pin<4U>;

        }    // namespace pins

    // 1. Pins for I/O Hardware

    // Onboard LED (debugging)
    using led = digital_pin<13U>;

    // Red LED
    using statusRed = digital_pin<6U>;

    // Green LED
    using statusGreen = digital_pin<5U>;

    // 2. Pins for Encoder, Motor and Wheel

    using right_motor = motor<pins::in1, pins::in2>;
    using left_motor = motor<pins::in3, pins::in4>;
    using en1 = digital_pin<11U>;
    using en2 = digital_pin<10U>;

    using left_wheel = wheel<pins::left_encoder_a, pins::left_encoder_b>;
    using right_wheel = wheel<pins::right_encoder_a, pins::right_encoder_b>;

    using left_encoder = encoder<pins::left_encoder_a, pins::left_encoder_b>;
    using right_encoder = encoder<pins::right_encoder_a, pins::right_encoder_b>;

    }    // namespace hardware
