#pragma once
#include "../chassis/tankChassis.hpp"
#include "../hardware/imu.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "odomSource.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include <cmath>
#include <errno.h>

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a set of differential wheels
     */
    class DifferentialWheelOdometry : public OdomSource, public Runnable
    {
    public:
        /**
         * Creates a new tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param chassis The chassis to use for odometry.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        DifferentialWheelOdometry(TankChassis &chassis,
                                  const double wheelRadius,
                                  const double wheelBase)
            : chassis(&chassis),
              wheelRadius(wheelRadius),
              wheelBase(wheelBase)
        {
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Creates a new odom wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param leftSensor The left rotation sensor.
         * @param rightSensor The right rotation sensor.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        DifferentialWheelOdometry(RotationSensor &leftSensor,
                                  RotationSensor &rightSensor,
                                  const double wheelRadius,
                                  const double wheelBase)
            : leftSensor(&leftSensor),
              rightSensor(&rightSensor),
              wheelRadius(wheelRadius),
              wheelBase(wheelBase)
        {
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the odometry from the left and right rotational values.
         * @param leftRotations The left wheel rotations.
         * @param rightRotations The right wheel rotations.
         */
        void _updateRotations(double leftRotations, double rightRotations)
        {
            // Get Delta Time
            uint32_t deltaT = lastUpdateTimestamp - pros::millis();
            lastUpdateTimestamp = pros::millis();

            // Get Distance
            double left = leftRotations * 2 * M_PI * wheelRadius;
            double right = rightRotations * 2 * M_PI * wheelRadius;

            // Get Delta Distance
            double deltaLeft = left - lastLeft;
            double deltaRight = right - lastRight;
            lastLeft = left;
            lastRight = right;

            // Calculate Delta Distance
            double deltaDistance = (deltaLeft + deltaRight) / 2;
            double deltaRotation = (deltaLeft - deltaRight) / wheelBase;

            // Calculate Delta X and Y
            double deltaX = deltaDistance * std::cos(currentPose.rotation + deltaRotation / 2);
            double deltaY = deltaDistance * std::sin(currentPose.rotation + deltaRotation / 2);

            // Update X, Y, and Rotation
            currentPose.x += deltaX;
            currentPose.y += deltaY;
            currentPose.rotation += deltaRotation;

            // Update IMU
            _updateIMU();
        }

        /**
         * Updates the odometry from the left and right rotation sensors.
         * @param leftSensor The left rotation sensor.
         * @param rightSensor The right rotation sensor.
         */
        void _updateOdomWheels(RotationSensor &leftSensor, RotationSensor &rightSensor)
        {
            double leftRotations = (leftSensor.getAngle() / (2 * M_PI)) / ticksPerRevolution;
            double rightRotations = (rightSensor.getAngle() / (2 * M_PI)) / ticksPerRevolution;
            _updateRotations(leftRotations, rightRotations);
        }

        /**
         * Updates the odometry from the left and right motor encoders of a tank chassis.
         */
        void _updateChassis(TankChassis &chassis)
        {
            double leftPosition = chassis.getLeftMotors().getPosition() / ticksPerRevolution;
            double rightPosition = chassis.getRightMotors().getPosition() / ticksPerRevolution;
            _updateRotations(leftPosition, rightPosition);
        }

        void update() override
        {
            if (chassis != nullptr)
                _updateChassis(*chassis);
            else if (leftSensor != nullptr && rightSensor != nullptr)
                _updateOdomWheels(*leftSensor, *rightSensor);
        }

        /**
         * Updates the rotation from an IMU specified in `useIMU`.
         */
        void _updateIMU()
        {
            if (imu == nullptr)
                return;
            double heading = imu->getHeading();
            if (heading != PROS_ERR_F)
                currentPose.rotation = heading;
        }

        /**
         * Enables the IMU for the odometry.
         * @param imu The IMU to use.
         */
        void useIMU(IMU &imu)
        {
            this->imu = &imu;
        }

        /**
         * Gets the current pose of the robot.
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;
            if (imu != nullptr)
                imu->setHeading(pose.rotation);
        }

        /**
         * Sets the number of ticks per revolution of the wheels.
         * Used to pass gear ratios to the odometry system.
         * @param ticksPerRevolution The number of encoder ticks per revolution of the wheels.
         */
        void setTicksPerRevolution(double ticksPerRevolution)
        {
            this->ticksPerRevolution = ticksPerRevolution;
        }

    private:
        const double wheelRadius;
        const double wheelBase;

        TankChassis *chassis = nullptr;
        RotationSensor *leftSensor = nullptr;
        RotationSensor *rightSensor = nullptr;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        double lastLeft = 0;
        double lastRight = 0;
        double lastVertical = 0;
        double lastHorizontal = 0;

        double ticksPerRevolution = 1.0;

        // IMU
        IMU *imu = nullptr;
    };
}