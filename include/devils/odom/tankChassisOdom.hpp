#pragma once
#include "differentialWheelOdom.hpp"
#include "../utils/runnable.hpp"

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a tank chassis.
     */
    class TankChassisOdom : public DifferentialWheelOdom, public Runnable
    {
    public:
        /**
         * Creates a new tank wheel odometry system.
         * Position is calculated using the left and right encoder values.
         * @param chassis The chassis to use for odometry.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        TankChassisOdom(TankChassis &chassis,
                        const double wheelRadius,
                        const double wheelBase)
            : chassis(chassis),
              DifferentialWheelOdom(wheelRadius, wheelBase)
        {
        }

        void onUpdate() override
        {
            // Process differential odometry
            double leftPosition = chassis.getLeftMotors().getPosition() / ticksPerRevolution;
            double rightPosition = chassis.getRightMotors().getPosition() / ticksPerRevolution;
            DifferentialWheelOdom::update(leftPosition, rightPosition);

            // Apply IMU
            if (imu != nullptr)
            {
                Pose currentPose = getPose();
                currentPose.rotation = imu->getHeading();
                setPose(currentPose);
            }
        }

        /**
         * Sets the number of encoder ticks per full revolution of the wheels.
         */
        void setTicksPerRevolution(double ticksPerRevolution)
        {
            this->ticksPerRevolution = ticksPerRevolution;
        }

        /**
         * Sets the IMU to use for odometry.
         * @param imu The IMU to use.
         */
        void useIMU(IMU *imu)
        {
            this->imu = imu;
        }

    private:
        double ticksPerRevolution = 300.0 * (48.0 / 36.0); // ticks
        TankChassis &chassis;
        IMU *imu;
    };
}