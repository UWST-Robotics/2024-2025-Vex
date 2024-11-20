#pragma once

#include "../utils/runnable.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../hardware/imu.hpp"

namespace devils
{
    /**
     * Represents an odometry system using a set of perpendicular rotation sensors.
     * If the sensors are parallel, use `ParallelSensorOdometry` instead.
     */
    class PerpendicularSensorOdometry : public OdomSource, public Runnable
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
        void onUpdate()
        {
            // Get Sensor Angles in Radians
            double verticalRads = verticalSensor.getAngle();
            double horizontalRads = horizontalSensor.getAngle();

            // Get Delta Time
            uint32_t deltaT = lastUpdateTimestamp - pros::millis();
            lastUpdateTimestamp = pros::millis();

            // Get Circumference Distance
            double vertical = verticalRads * wheelRadius;
            double horizontal = verticalRads * wheelRadius;

            // Get Delta Distance
            double deltaVertical = vertical - lastVertical;
            double deltaHorizontal = horizontal - lastHorizontal;
            lastVertical = vertical;
            lastHorizontal = horizontal;

            // Append to Pose
            currentPose.x += deltaVertical;
            currentPose.y += deltaHorizontal;
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
        }

        /**
         * Sets the IMU to use for odometry.
         * @param imu The IMU to use for odometry.
         */
        void useIMU(IMU *imu)
        {
            this->imu = imu;
        }

    private:
        const double wheelRadius;

        RotationSensor &verticalSensor;
        RotationSensor &horizontalSensor;
        IMU *imu = nullptr;

        Pose currentPose = Pose();
        uint32_t lastUpdateTimestamp = 0;

        double lastLeft = 0;
        double lastRight = 0;
        double lastVertical = 0;
        double lastHorizontal = 0;
    };
}