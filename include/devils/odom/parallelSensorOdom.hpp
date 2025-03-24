#pragma once

#include "differentialWheelOdom.hpp"
#include "../utils/runnable.hpp"
#include "poseVelocityCalculator.hpp"

#define M_PI 3.14159265358979323846

namespace devils
{
    /**
     * Represents an odometry system using a set of parallel rotation sensors.
     * If the sensors are perpendicular, use `PerpendicularSensorOdometry` instead.
     */
    class ParallelSensorOdometry : public DifferentialWheelOdom, public Runnable
    {
    public:
        /**
         * Creates an odometry system using a set of parallel rotation sensors.
         * @param leftSensor The left rotation sensor.
         * @param rightSensor The right rotation sensor.
         * @param wheelRadius The radius of the wheels in inches.
         * @param wheelBase The distance between the wheels in inches.
         */
        ParallelSensorOdometry(RotationSensor &leftSensor,
                               RotationSensor &rightSensor,
                               const double wheelRadius,
                               const double wheelBase)
            : leftSensor(leftSensor),
              rightSensor(rightSensor),
              DifferentialWheelOdom(wheelRadius, wheelBase)
        {
        }

        void onUpdate() override
        {
            double leftPosition = (leftSensor.getAngle() / (2 * M_PI)) / ticksPerRevolution;
            double rightPosition = (rightSensor.getAngle() / (2 * M_PI)) / ticksPerRevolution;
            DifferentialWheelOdom::update(leftPosition, rightPosition);
        }

        /**
         * Sets the number of encoder ticks per full revolution of the wheels.
         */
        void setTicksPerRevolution(double ticksPerRevolution)
        {
            this->ticksPerRevolution = ticksPerRevolution;
        }

    private:
        double ticksPerRevolution = 300.0 * (48.0 / 36.0); // ticks
        RotationSensor &leftSensor;
        RotationSensor &rightSensor;
    };
}