#pragma once

#include "smartMotorGroup.hpp"
#include "vexbridge/vexBridge.h"

namespace devils
{
    /**
     * Controls 2 sets of motors symmetrically using encoders.
     * Acts like a differential drive, but uses encoders to adjust the speed of each side.
     * The motor that is ahead will be slowed down proportionally to its distance ahead.
     */
    class SymmetricControl
    {
    public:
        SymmetricControl(SmartMotorGroup &leftMotors, SmartMotorGroup &rightMotors)
            : leftMotors(leftMotors), rightMotors(rightMotors)
        {
        }

        /**
         * Resets the offsets of the left and right motors.
         */
        void resetOffsets()
        {
            leftOffset = leftMotors.getPosition();
            rightOffset = rightMotors.getPosition();
        }

        /**
         * Drives the motors in a symmetric manner.
         * @param speed The speed to drive the motors at, from -1 to 1.
         */
        void drive(double speed, double horizontalSpeed)
        {
            // Clamp the speed to the range [-1, 1]
            speed = std::clamp(speed, -1.0, 1.0);
            horizontalSpeed = std::clamp(horizontalSpeed, -1.0, 1.0);

            // Apply horizontal deadzone
            if (std::abs(horizontalSpeed) < HORIZONTAL_DEADZONE)
                horizontalSpeed = 0.0;

            // Adjust encoder offsets
            leftOffset -= horizontalSpeed * HORIZONTAL_ENCODER_SPEED;
            rightOffset += horizontalSpeed * HORIZONTAL_ENCODER_SPEED;

            // Fetch the encoder values and subtract the offsets
            double leftEncoder = leftMotors.getPosition() - leftOffset;
            double rightEncoder = rightMotors.getPosition() - rightOffset;

            // Check which motor is behind
            bool leftMotorBehind = leftEncoder < rightEncoder;
            bool reverse = speed < 0;
            if (reverse)
                leftMotorBehind = !leftMotorBehind;

            // Calculate the speed scaling factor
            double behindEncoder = leftMotorBehind ? leftEncoder : rightEncoder;
            double aheadEncoder = leftMotorBehind ? rightEncoder : leftEncoder;
            double deltaEncoder = leftEncoder - rightEncoder;

            double speedScale = 1.0 - (std::abs(deltaEncoder) / ENCODER_MAX_OFFSET);
            speedScale = std::clamp(speedScale, 0.0, 1.0);

            // Calculate the horizontal speed scaling factor
            double horizontalScale = deltaEncoder *
                                     HORIZONTAL_MULTIPLIER *
                                     std::abs(horizontalSpeed);

            // Drive the motors with the calculated speed
            if (leftMotorBehind)
            {
                leftMotors.moveVoltage(speed - horizontalScale);
                rightMotors.moveVoltage(speed * speedScale + horizontalScale);
            }
            else
            {
                leftMotors.moveVoltage(speed * speedScale - horizontalScale);
                rightMotors.moveVoltage(speed + horizontalScale);
            }
        }

    private:
        static constexpr double HORIZONTAL_DEADZONE = 0.5;     // deadzone for the joystick
        static constexpr double HORIZONTAL_MULTIPLIER = 0.005; // %
        static constexpr double HORIZONTAL_ENCODER_SPEED = 3;  // ticks per iteration
        static constexpr double ENCODER_MAX_OFFSET = 100;      // ticks
        static constexpr bool IS_REVERSED = true;              // true if the motors are reversed

        SmartMotorGroup &leftMotors;
        SmartMotorGroup &rightMotors;

        double leftOffset = 0.0;
        double rightOffset = 0.0;
    };
}