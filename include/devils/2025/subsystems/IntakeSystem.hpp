#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents the intake arm and claw system of the robot.
     */
    class IntakeSystem
    {
    public:
        /// @brief Represents the possible positions of the intake arm.
        enum ArmPosition
        {
            BOTTOM_RING,    // Grabs rings off the ground
            INTAKE,         // Slightly elevated to allow for intake
            THIRD_RING,     // Grabs the 3rd ring off a stack, used for autonomous
            FOURTH_RING,    // Grabs the 4th ring off a stack, used for autonomous
            ALLIANCE_STAKE, // Raises the arm to the shorter alliance (red/blue) stakes
            NEUTRAL_STAKE,  // Raises the arm to the taller neutral stakes
        };

        /**
         * Creates a new instance of the intake system.
         * @param grabberPneumatic The pneumatic system controlling the grabbing claws.
         * @param armMotors The motors controlling the arm.
         * @param rotationSensor The sensor measuring the rotation of the arm.
         */
        IntakeSystem(ADIPneumatic &grabberPneumatic, SmartMotorGroup &armMotors, RotationSensor &rotationSensor)
            : grabberPneumatic(grabberPneumatic),
              armMotors(armMotors),
              rotationSensor(rotationSensor)
        {
        }

        /**
         * Sets the target position of the intake arm.
         * @param position The position to set the arm to.
         */
        void setArmPosition(ArmPosition position)
        {
            this->targetPosition = position;
        }

        /**
         * Gets the target position of the intake arm.
         * @return The target position of the arm.
         */
        ArmPosition getArmPosition()
        {
            return targetPosition;
        }

        /**
         * Moves the arm to the target position.
         * Should be called every interval to update the state of the arm.
         */
        void moveArmToPosition()
        {
            double targetAngle = convertPositionToAngle(this->targetPosition);
            moveArmToAngle(targetAngle);
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

        /**
         * Stops all motors in the arm.
         */
        void stop()
        {
            for (auto motor : armMotors.getMotors())
                motor->moveVoltage(0);
        }

    protected:
        /**
         * Converts the arm position to target angle.
         * @param position The position to convert.
         * @return The cooresponding angle in radians.
         */
        double convertPositionToAngle(ArmPosition position)
        {
            switch (position)
            {
            case BOTTOM_RING:
                return BOTTOM_RING_POSITION;
            case INTAKE:
                return INTAKE_POSITION;
            case THIRD_RING:
                return THIRD_RING_POSITION;
            case FOURTH_RING:
                return FOURTH_RING_POSITION;
            case ALLIANCE_STAKE:
                return ALLIANCE_STAKE_POSITION;
            case NEUTRAL_STAKE:
                return NEUTRAL_STAKE_POSITION;
            }
        }

        /**
         * Moves the arm to the specified angle.
         * Should be called every interval to update the state of the arm.
         * @param angle The target angle of the arm in radians.
         */
        void moveArmToAngle(double angle)
        {
            for (auto motor : armMotors.getMotors())
                moveMotorToAngle(motor.get(), angle);
        }

        /**
         * Moves the motor to the specified angle.
         * Should be called every interval to update the state of the motor.
         * @param motor The motor to move.
         * @param angle The target angle of the motor in radians.
         */
        void moveMotorToAngle(SmartMotor *motor, double angle)
        {
            // Fail if the sensor is not connected
            if (!rotationSensor.isConnected())
            {
                motor->moveVoltage(0);
                return;
            }

            double currentPosition = rotationSensor.getAngle();
            double error = Units::diffRad(angle, currentPosition);

            double speed = armPID.update(error);
            speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);
            // double speed = Math::trapezoidProfile(
            //     1,
            //     error,
            //     1,
            //     DECEL_DISTANCE,
            //     0,
            //     0,
            //     MAX_SPEED);

            motor->moveVoltage(speed);
        }

    private:
        static constexpr double BOTTOM_RING_POSITION = 0;        // rad
        static constexpr double INTAKE_POSITION = -0.15;         // rad
        static constexpr double THIRD_RING_POSITION = -0.33;     // rad
        static constexpr double FOURTH_RING_POSITION = -0.35;    // rad
        static constexpr double ALLIANCE_STAKE_POSITION = -1.05; // rad
        static constexpr double NEUTRAL_STAKE_POSITION = -1.6;   // rad

        static constexpr double DECEL_DISTANCE = M_PI * 0.2; // rad
        static constexpr double MAX_SPEED = 0.3;             // %
        static constexpr double MIN_SPEED = -0.6;            // %

        // State
        ArmPosition targetPosition = BOTTOM_RING;
        PIDController armPID = PIDController(1.5, 0.0, 40);

        // Hardware
        ADIPneumatic &grabberPneumatic;
        SmartMotorGroup &armMotors;
        RotationSensor &rotationSensor;
    };
}