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

namespace devils
{
    /**
     * Represents an odometry system using vertical and horizontal tracking wheels
     */
    class TrackingWheelOdometry : public OdomSource
    {
    public:
        /**
         * Creates a new tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        TrackingWheelOdometry(const double wheelRadius)
            : wheelRadius(wheelRadius)
        {
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the odometry from vertical and horizontal tracking wheels.
         * @param verticalEncoder The vertical encoder rotations.
         * @param horizontalEncoder The horizontal encoder rotations.
         */
        void update(int verticalEncoder, int horizontalEncoder)
        {
            // Get Delta Time
            uint32_t deltaT = lastUpdateTimestamp - pros::millis();
            lastUpdateTimestamp = pros::millis();

            // Get Distance
            double vertical = verticalEncoder * 2 * M_PI * wheelRadius;
            double horizontal = horizontalEncoder * 2 * M_PI * wheelRadius;

            // Get Delta Distance
            double deltaVertical = vertical - lastVertical;
            double deltaHorizontal = horizontal - lastHorizontal;
            lastVertical = vertical;
            lastHorizontal = horizontal;

            // Run Computationally Expensive Trig Functions
            double sinTheta = std::sin(currentPose.rotation);
            double cosTheta = std::cos(currentPose.rotation);

            // Calculate Delta X and Y
            double deltaX = deltaVertical * cosTheta + deltaHorizontal * sinTheta;
            double deltaY = deltaVertical * sinTheta - deltaHorizontal * cosTheta;

            // Update X, Y, and Rotation
            currentPose.x += deltaX;
            currentPose.y += deltaY;

            // Update IMU
            _updateIMU();
        }

        /**
         * Updates the odometry from vertical and horizontal tracking wheels.
         * @param verticalSensor The vertical tracking sensor.
         * @param horizontalSensor The horizontal tracking sensor.
         */
        void update(RotationSensor &verticalSensor, RotationSensor &horizontalSensor)
        {
            double vertical = verticalSensor.getAngle() / (M_PI * 2);
            double horizontal = horizontalSensor.getAngle() / (M_PI * 2);
            update(vertical, horizontal);
        }

        /**
         * Updates the rotation from an IMU specified in `useIMU`.
         */
        void _updateIMU()
        {
            if (imu == nullptr)
                return;

            double imuHeading = imu->getHeading();
            if (imuHeading != PROS_ERR_F)
                currentPose.rotation = imuHeading;
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

    private:
        const double wheelRadius;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        double lastLeft = 0;
        double lastRight = 0;
        double lastVertical = 0;
        double lastHorizontal = 0;

        // IMU
        IMU *imu = nullptr;
    };
}