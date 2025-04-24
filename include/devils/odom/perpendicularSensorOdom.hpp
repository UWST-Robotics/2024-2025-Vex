#pragma once

#include "../utils/runnable.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../hardware/structs/gyro.h"
#include "poseVelocityCalculator.hpp"

namespace devils
{
    /**
     * Represents an odometry system using a set of perpendicular rotation sensors.
     * If the sensors are parallel, use `ParallelSensorOdometry` instead.
     */
    class PerpendicularSensorOdometry : public OdomSource, public Runnable, public PoseVelocityCalculator
    {
    public:
        /**
         * Creates an odometry system using a set of perpendicular rotation sensors.
         * @param verticalSensor The vertical tracking sensor.
         * @param horizontalSensor The horizontal tracking sensor.
         * @param wheelRadius The radius of the wheels in inches.
         */
        PerpendicularSensorOdometry(
            RotationSensor &verticalSensor,
            RotationSensor &horizontalSensor,
            const double wheelRadius)
            : verticalSensor(verticalSensor),
              horizontalSensor(horizontalSensor),
              wheelRadius(wheelRadius)
        {
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the odometry from vertical and horizontal tracking wheels.
         */
        void onUpdate() override
        {
            // Get Sensor Angles in Degrees
            double verticalAngle = verticalSensor.getAngle();
            bool isVerticalSensorError = (errno != 0);

            double horizontalAngle = horizontalSensor.getAngle();
            bool isHorizontalSensorError = (errno != 0);

            // Get Delta Time
            uint32_t deltaT = lastUpdateTimestamp - pros::millis();
            lastUpdateTimestamp = pros::millis();

            // Calculate arc length
            // Arc Length = r * theta
            double vertical = verticalAngle * wheelRadius;
            double horizontal = horizontalAngle * wheelRadius;

            // Update IMU
            // Also calculate the change in rotation
            double deltaRotation = 0;
            if (imu != nullptr)
            {
                double heading = imu->getHeading();
                if (errno == 0)
                {
                    deltaRotation = heading - lastRotation;
                    lastRotation = heading;
                    currentPose.rotation = heading;
                }
            }

            // Get Delta Distance
            double deltaVertical = vertical - lastVertical;
            double deltaHorizontal = horizontal - lastHorizontal;

            // Check for sensor errors
            if (!isVerticalSensorError)
                lastVertical = vertical;
            if (!isHorizontalSensorError)
                lastHorizontal = horizontal;

            // Apply Sensor Offsets
            if (verticalSensorOffset != nullptr &&
                horizontalSensorOffset != nullptr)
            {
                // Calculate radius of rotation for each sensor
                // We only care about the x and y components since the sensors are perpendicular to the other axis
                double verticalOffsetRadius = verticalSensorOffset->x;
                double horizontalOffsetRadius = horizontalSensorOffset->y;

                // Calculate Arc Length
                // Arc Length = r * theta
                double verticalArcLength = verticalOffsetRadius * deltaRotation;
                double horizontalArcLength = horizontalOffsetRadius * deltaRotation;

                // Subtract Arc Length
                deltaVertical -= verticalArcLength;
                deltaHorizontal -= horizontalArcLength;
            }

            // Check for sensor errors
            if (isVerticalSensorError)
                deltaVertical = 0;
            if (isHorizontalSensorError)
                deltaHorizontal = 0;

            // Calculate trigonometric values
            double rotation = currentPose.rotation;
            double sin = std::sin(rotation);
            double cos = std::cos(rotation);

            double deltaX = deltaVertical * cos + deltaHorizontal * sin;
            double deltaY = deltaVertical * sin - deltaHorizontal * cos;

            // Update Pose
            currentPose.x += deltaX;
            currentPose.y += deltaY;

            // Update Velocity
            PoseVelocityCalculator::updateVelocity(currentPose);
        }

        /**
         * Gets the current pose of the robot.
         */
        Pose getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose pose) override
        {
            currentPose = pose;

            if (imu != nullptr)
                imu->setHeading(pose.rotation);
        }

        /**
         * Sets the IMU to use for odometry.
         * @param imu The IMU to use for odometry.
         */
        void useIMU(IGyro *imu)
        {
            this->imu = imu;
        }

        /**
         * Sets the sensor offsets for the odometry system.
         * Accounts for the difference in sensor placement on the robot.
         * @param verticalSensorOffset The offset for the vertical sensor relative to the robot's center of rotation.
         * @param horizontalSensorOffset The offset for the horizontal sensor relative to the robot's center of rotation.
         */
        void setSensorOffsets(Vector2 &verticalSensorOffset,
                              Vector2 &horizontalSensorOffset)
        {
            this->verticalSensorOffset = &verticalSensorOffset;
            this->horizontalSensorOffset = &horizontalSensorOffset;
        }

        PoseVelocity getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

    private:
        const double wheelRadius;
        RotationSensor &verticalSensor;
        RotationSensor &horizontalSensor;

        IGyro *imu = nullptr;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        double lastVertical = 0;
        double lastHorizontal = 0;
        double lastRotation = 0;

        Vector2 *verticalSensorOffset = nullptr;
        Vector2 *horizontalSensorOffset = nullptr;
    };
}