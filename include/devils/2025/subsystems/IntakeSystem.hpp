#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents the intake arm assembly of the robot.
     */
    class IntakeSystem
    {
    public:
        /// @brief Represents the possible positions of the intake arm.
        enum ArmPosition
        {
            BOTTOM_RING,
            THIRD_RING,
            ALLIANCE_STAKE,
            NEUTRAL_STAKE,
        };

        /**
         * Creates a new instance of the intake system.
         * @param grabberPneumatic The pneumatic system controlling the grabbing claws.
         * @param armMotors The motors controlling the arm.
         */
        IntakeSystem(ADIPneumatic &grabberPneumatic, SmartMotorGroup &armMotors)
            : grabberPneumatic(grabberPneumatic),
              armMotors(armMotors)
        {
        }

        /**
         * Sets the position of the intake arm.
         * @param position The position to set the arm to.
         */
        void moveArmToPosition(ArmPosition position)
        {
            this->targetPosition = position;
            double targetEncoderTicks = convertPositionToEncoderTicks(position);
            moveArmToEncoderTicks(targetEncoderTicks);
        }

        /**
         * Checks if the arm is at the target position.
         * @return True if the arm is at the target position, false otherwise.
         */
        bool checkArmAtPosition()
        {
            double currentPosition = armMotors.getPosition();
            double targetPosition = convertPositionToEncoderTicks(this->targetPosition);
            return std::abs(currentPosition - targetPosition) < ARM_POSITION_THRESHOLD;
        }

        /**
         * Sets whether the claw is grabbed.
         * @param isGrabbed True if the claw is grabbed, false otherwise.
         */
        void setClawGrabbed(bool isGrabbed)
        {
            grabberPneumatic.setExtended(isGrabbed);
        }

        /**
         * Gets whether the claw is grabbed.
         * @return True if the claw is grabbed, false otherwise.
         */
        bool getClawGrabbed()
        {
            return grabberPneumatic.getExtended();
        }

    protected:
        /**
         * Converts the arm position to encoder ticks.
         * @param position The position to convert.
         * @return The position in encoder ticks.
         */
        double convertPositionToEncoderTicks(ArmPosition position)
        {
            switch (position)
            {
            case BOTTOM_RING:
                return BOTTOM_RING_POSITION;
            case THIRD_RING:
                return THIRD_RING_POSITION;
            case ALLIANCE_STAKE:
                return ALLIANCE_STAKE_POSITION;
            case NEUTRAL_STAKE:
                return NEUTRAL_STAKE_POSITION;
            }
        }

        /**
         * Moves the arm to the specified position.
         * @param targetPosition The target position of the arm in encoder ticks.
         */
        void moveArmToEncoderTicks(double encoderTicks)
        {
            for (auto motor : armMotors.getMotors())
                moveMotorToPosition(motor.get(), encoderTicks);
        }

        /**
         * Moves the motor to the specified position.
         * @param motor The motor to move.
         * @param encoderTicks The target position of the motor in encoder ticks.
         */
        void moveMotorToPosition(SmartMotor *motor, double encoderTicks)
        {
            double currentPosition = motor->getPosition();
            double error = encoderTicks - currentPosition;

            double speed = Math::trapezoidProfile(
                1,
                error,
                1,
                DECEL_DISTANCE,
                0,
                0,
                MAX_SPEED);

            motor->moveVoltage(speed);
        }

    private:
        static constexpr double BOTTOM_RING_POSITION = 0;      // ticks
        static constexpr double THIRD_RING_POSITION = 200;     // ticks
        static constexpr double ALLIANCE_STAKE_POSITION = 500; // ticks
        static constexpr double NEUTRAL_STAKE_POSITION = 600;  // ticks

        static constexpr double ARM_POSITION_THRESHOLD = 10; // ticks

        static constexpr double DECEL_DISTANCE = 90; // ticks
        static constexpr double MAX_SPEED = 0.8;     // %

        // State
        ArmPosition targetPosition;

        // Hardware
        ADIPneumatic &grabberPneumatic;
        SmartMotorGroup &armMotors;
    };
}