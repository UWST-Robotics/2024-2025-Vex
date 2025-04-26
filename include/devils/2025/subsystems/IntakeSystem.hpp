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
            SECOND_RING,    // Grabs the 2nd ring off a stack, used for autonomous
            FOURTH_RING,    // Grabs the 4th ring off a stack, used for autonomous
            ALLIANCE_STAKE, // Raises the arm to the shorter alliance (red/blue) stakes
            NEUTRAL_STAKE,  // Raises the arm to the taller neutral stakes
        };

        /// @brief Represents the angles of the intake arm for each position.
        struct ArmPositionAngles
        {
            double bottomRing = 50;
            double intake = 130;
            double secondRing = 200;
            double fourthRing = 400;
            double allianceStake = 650;
            double neutralStake = 1000;
        };

        /**
         * Creates a new instance of the intake system.
         * @param grabberPneumatic The pneumatic system controlling the grabbing claws.
         * @param armMotors The motors controlling the arm.
         * @param rotationSensor The sensor measuring the rotation of the arm.
         */
        IntakeSystem(ADIPneumatic &grabberPneumatic, SmartMotorGroup &armMotors)
            : grabberPneumatic(grabberPneumatic),
              armMotors(armMotors)
        {
            armMotors.setPosition(0);
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

        /**
         * Disables the speed clamp for the arm.
         * @param isDisabled True to disable the speed clamp, false to enable it.
         */
        void disableSpeedClamp(bool isDisabled = true)
        {
            isSpeedClampDisabled = isDisabled;
        }

        /**
         * Sets the parameters for the arm PID controller.
         * @param params The parameters to set.
         */
        void setArmPID(PIDController::Options params)
        {
            armPID = PIDController(params);
        }

        /**
         * Sets the arm position angles.
         * @param angles The angles to set.
         */
        void setArmPositions(ArmPositionAngles angles)
        {
            armPositionAngles = angles;
        }

    protected:
        /**
         * Converts the arm position to target angle.
         * @param position The position to convert.
         * @return The cooresponding angle in encoder positions.
         */
        double convertPositionToAngle(ArmPosition position)
        {
            switch (position)
            {
            case BOTTOM_RING:
                return armPositionAngles.bottomRing;
            case INTAKE:
                return armPositionAngles.intake;
            case SECOND_RING:
                return armPositionAngles.secondRing;
            case FOURTH_RING:
                return armPositionAngles.fourthRing;
            case ALLIANCE_STAKE:
                return armPositionAngles.allianceStake;
            case NEUTRAL_STAKE:
                return armPositionAngles.neutralStake;
            }
            return 0;
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
            // Get the current position of the motor
            double currentPosition = motor->getPosition();
            double error = angle - currentPosition;

            // PID Control
            double speed = armPID.update(error);

            // Clamp speed to avoid slamming
            if (!isSpeedClampDisabled)
                speed = std::clamp(speed, MIN_SPEED, MAX_SPEED);

            motor->moveVoltage(speed);
        }

    private:
        static constexpr double MAX_SPEED = 0.8;  // %
        static constexpr double MIN_SPEED = -0.6; // %

        // State
        ArmPositionAngles armPositionAngles = ArmPositionAngles();
        ArmPosition targetPosition = BOTTOM_RING;
        PIDController armPID = PIDController(0.003, 0, 0);
        bool isSpeedClampDisabled = false;

        // Hardware
        ADIPneumatic &grabberPneumatic;
        SmartMotorGroup &armMotors;
    };
}